// Package serial provides serial port communication for Klipper MCU connections.
package serial

import (
	"errors"
	"fmt"
	"io"
	"os"
	"path/filepath"
	"runtime"
	"sort"
	"strings"
	"sync"
	"time"
	"unsafe"

	"golang.org/x/sys/unix"
)

// Common errors
var (
	ErrNotConnected = errors.New("serial: not connected")
	ErrTimeout      = errors.New("serial: operation timed out")
	ErrClosed       = errors.New("serial: port closed")
)

// Config holds serial port configuration.
type Config struct {
	// Device path (e.g., /dev/ttyUSB0, /dev/ttyACM0)
	Device string

	// Baud rate (default: 250000)
	BaudRate int

	// Connection timeout (default: 60 seconds)
	ConnectTimeout time.Duration

	// Read timeout for individual operations (default: 5 seconds)
	ReadTimeout time.Duration

	// RTS/DTR control
	RTSOnConnect bool
	DTROnConnect bool
}

// DefaultConfig returns a Config with default values.
func DefaultConfig() Config {
	return Config{
		BaudRate:       250000,
		ConnectTimeout: 60 * time.Second,
		ReadTimeout:    5 * time.Second,
		RTSOnConnect:   true,
		DTROnConnect:   true,
	}
}

// Port represents a serial port connection.
type Port struct {
	mu         sync.Mutex
	fd         int
	device     string
	config     Config
	closed     bool
	oldTermios *unix.Termios
	isSocket   bool    // true if connected via Unix socket (e.g., Linux MCU simulator)
	isCAN      bool    // true if this is a CAN bus connection wrapper
	canbus     *CANBus // CAN bus connection (when isCAN is true)
}

// ListPorts returns a list of available serial port device paths.
func ListPorts() ([]string, error) {
	var ports []string

	// Common serial device patterns by OS
	var patterns []string
	switch runtime.GOOS {
	case "linux":
		patterns = []string{
			"/dev/ttyUSB*",
			"/dev/ttyACM*",
			"/dev/ttyS*",
			"/dev/serial/by-id/*",
		}
	case "darwin":
		patterns = []string{
			"/dev/tty.usbserial*",
			"/dev/tty.usbmodem*",
			"/dev/cu.usbserial*",
			"/dev/cu.usbmodem*",
		}
	default:
		return nil, fmt.Errorf("serial: unsupported platform %s", runtime.GOOS)
	}

	for _, pattern := range patterns {
		matches, err := filepath.Glob(pattern)
		if err != nil {
			continue
		}
		for _, m := range matches {
			// Resolve symlinks (especially for /dev/serial/by-id/)
			resolved, err := filepath.EvalSymlinks(m)
			if err != nil {
				resolved = m
			}
			// Check if already in list
			found := false
			for _, p := range ports {
				if p == resolved {
					found = true
					break
				}
			}
			if !found {
				ports = append(ports, resolved)
			}
		}
	}

	sort.Strings(ports)
	return ports, nil
}

// Open opens a serial port with the given configuration.
func Open(cfg Config) (*Port, error) {
	if cfg.Device == "" {
		return nil, errors.New("serial: device path required")
	}
	if cfg.BaudRate == 0 {
		cfg.BaudRate = 250000
	}
	if cfg.ConnectTimeout == 0 {
		cfg.ConnectTimeout = 60 * time.Second
	}
	if cfg.ReadTimeout == 0 {
		cfg.ReadTimeout = 5 * time.Second
	}

	// Open the device
	fd, err := unix.Open(cfg.Device, unix.O_RDWR|unix.O_NOCTTY|unix.O_NONBLOCK, 0)
	if err != nil {
		return nil, fmt.Errorf("serial: open %s: %w", cfg.Device, err)
	}

	// Get current termios settings
	oldTermios, err := unix.IoctlGetTermios(fd, ioctlGetTermios)
	if err != nil {
		unix.Close(fd)
		return nil, fmt.Errorf("serial: get termios: %w", err)
	}

	// Configure port
	termios := *oldTermios

	// Input flags - disable all input processing
	termios.Iflag &^= unix.IGNBRK | unix.BRKINT | unix.PARMRK | unix.ISTRIP |
		unix.INLCR | unix.IGNCR | unix.ICRNL | unix.IXON | unix.IXOFF | unix.IXANY

	// Output flags - disable all output processing
	termios.Oflag &^= unix.OPOST

	// Control flags - 8N1
	termios.Cflag &^= unix.CSIZE | unix.PARENB | unix.PARODD | unix.CSTOPB
	termios.Cflag |= unix.CS8 | unix.CREAD | unix.CLOCAL

	// Local flags - raw mode
	termios.Lflag &^= unix.ECHO | unix.ECHONL | unix.ICANON | unix.ISIG | unix.IEXTEN

	// Set baud rate
	speed, customBaud, err := baudRateToSpeed(cfg.BaudRate)
	if err != nil {
		unix.Close(fd)
		return nil, err
	}
	setSpeed(&termios, speed)

	// Control characters
	termios.Cc[unix.VMIN] = 0  // Non-blocking read
	termios.Cc[unix.VTIME] = 1 // 100ms timeout per character

	// Apply settings
	if err := unix.IoctlSetTermios(fd, ioctlSetTermios, &termios); err != nil {
		unix.Close(fd)
		return nil, fmt.Errorf("serial: set termios: %w", err)
	}

	// On macOS, set custom baud rate using IOSSIOSPEED if needed
	if customBaud > 0 && runtime.GOOS == "darwin" {
		if err := setCustomBaudRate(fd, customBaud); err != nil {
			unix.Close(fd)
			return nil, fmt.Errorf("serial: set custom baud rate: %w", err)
		}
	}

	// Clear non-blocking flag after configuration
	if err := unix.SetNonblock(fd, false); err != nil {
		unix.Close(fd)
		return nil, fmt.Errorf("serial: set blocking: %w", err)
	}

	port := &Port{
		fd:         fd,
		device:     cfg.Device,
		config:     cfg,
		oldTermios: oldTermios,
	}

	// Set RTS/DTR
	if err := port.setModemControl(cfg.RTSOnConnect, cfg.DTROnConnect); err != nil {
		port.Close()
		return nil, fmt.Errorf("serial: set modem control: %w", err)
	}

	return port, nil
}

// OpenSocket connects to a Unix socket at the given path.
// This is used to connect to the Linux MCU simulator which communicates
// via a pseudo-tty exposed as a Unix socket.
func OpenSocket(socketPath string, timeout time.Duration) (*Port, error) {
	if socketPath == "" {
		return nil, errors.New("serial: socket path required")
	}
	if timeout == 0 {
		timeout = 60 * time.Second
	}

	// Create Unix socket
	fd, err := unix.Socket(unix.AF_UNIX, unix.SOCK_STREAM, 0)
	if err != nil {
		return nil, fmt.Errorf("serial: create socket: %w", err)
	}

	// Set up socket address
	addr := &unix.SockaddrUnix{Name: socketPath}

	// Try to connect with timeout
	deadline := time.Now().Add(timeout)
	var connectErr error
	for time.Now().Before(deadline) {
		connectErr = unix.Connect(fd, addr)
		if connectErr == nil {
			break
		}
		// Socket might not exist yet, wait and retry
		if errors.Is(connectErr, unix.ENOENT) || errors.Is(connectErr, unix.ECONNREFUSED) {
			time.Sleep(100 * time.Millisecond)
			continue
		}
		unix.Close(fd)
		return nil, fmt.Errorf("serial: connect to %s: %w", socketPath, connectErr)
	}
	if connectErr != nil {
		unix.Close(fd)
		return nil, fmt.Errorf("serial: connect timeout to %s: %w", socketPath, connectErr)
	}

	port := &Port{
		fd:       fd,
		device:   socketPath,
		config:   Config{ReadTimeout: 5 * time.Second},
		isSocket: true,
	}

	return port, nil
}

// OpenTCP connects to a TCP server at the given address (host:port).
// This is used to connect to the Linux MCU simulator running in Docker
// which exposes a TCP port for communication.
func OpenTCP(address string, timeout time.Duration) (*Port, error) {
	if address == "" {
		return nil, errors.New("serial: TCP address required")
	}
	if timeout == 0 {
		timeout = 60 * time.Second
	}

	// Create TCP socket
	fd, err := unix.Socket(unix.AF_INET, unix.SOCK_STREAM, 0)
	if err != nil {
		return nil, fmt.Errorf("serial: create TCP socket: %w", err)
	}

	// Parse address
	host, portStr, err := splitHostPort(address)
	if err != nil {
		unix.Close(fd)
		return nil, fmt.Errorf("serial: parse address %s: %w", address, err)
	}

	port, err := parsePort(portStr)
	if err != nil {
		unix.Close(fd)
		return nil, fmt.Errorf("serial: parse port %s: %w", portStr, err)
	}

	// Resolve host to IP
	ip, err := resolveHost(host)
	if err != nil {
		unix.Close(fd)
		return nil, fmt.Errorf("serial: resolve host %s: %w", host, err)
	}

	// Set up socket address
	addr := &unix.SockaddrInet4{Port: port}
	copy(addr.Addr[:], ip)

	// Try to connect with timeout
	deadline := time.Now().Add(timeout)
	var connectErr error
	for time.Now().Before(deadline) {
		connectErr = unix.Connect(fd, addr)
		if connectErr == nil {
			break
		}
		// Server might not be ready yet, wait and retry
		if errors.Is(connectErr, unix.ECONNREFUSED) {
			time.Sleep(100 * time.Millisecond)
			continue
		}
		unix.Close(fd)
		return nil, fmt.Errorf("serial: connect to %s: %w", address, connectErr)
	}
	if connectErr != nil {
		unix.Close(fd)
		return nil, fmt.Errorf("serial: connect timeout to %s: %w", address, connectErr)
	}

	p := &Port{
		fd:       fd,
		device:   address,
		config:   Config{ReadTimeout: 5 * time.Second},
		isSocket: true, // Treat TCP same as socket for I/O purposes
	}

	return p, nil
}

// splitHostPort splits an address of the form "host:port" into host and port.
func splitHostPort(address string) (string, string, error) {
	for i := len(address) - 1; i >= 0; i-- {
		if address[i] == ':' {
			return address[:i], address[i+1:], nil
		}
	}
	return "", "", errors.New("missing port in address")
}

// parsePort parses a port string to an integer.
func parsePort(s string) (int, error) {
	var port int
	for _, c := range s {
		if c < '0' || c > '9' {
			return 0, errors.New("invalid port number")
		}
		port = port*10 + int(c-'0')
	}
	if port == 0 || port > 65535 {
		return 0, errors.New("port out of range")
	}
	return port, nil
}

// resolveHost resolves a hostname to an IPv4 address.
func resolveHost(host string) ([]byte, error) {
	// Handle localhost specially
	if host == "localhost" || host == "127.0.0.1" {
		return []byte{127, 0, 0, 1}, nil
	}

	// Try to parse as IP address
	ip := make([]byte, 4)
	parts := 0
	val := 0
	for i := 0; i <= len(host); i++ {
		if i == len(host) || host[i] == '.' {
			if val > 255 {
				return nil, errors.New("invalid IP address")
			}
			ip[parts] = byte(val)
			parts++
			val = 0
		} else if host[i] >= '0' && host[i] <= '9' {
			val = val*10 + int(host[i]-'0')
		} else {
			return nil, errors.New("hostname resolution not supported, use IP address")
		}
	}
	if parts != 4 {
		return nil, errors.New("invalid IP address format")
	}
	return ip, nil
}

// IsSocket returns true if this port is connected via Unix socket or TCP.
func (p *Port) IsSocket() bool {
	p.mu.Lock()
	defer p.mu.Unlock()
	return p.isSocket
}

// Read reads up to len(buf) bytes from the port.
// Returns the number of bytes read and any error.
func (p *Port) Read(buf []byte) (int, error) {
	p.mu.Lock()
	if p.closed {
		p.mu.Unlock()
		return 0, ErrClosed
	}
	fd := p.fd
	timeout := p.config.ReadTimeout
	p.mu.Unlock()

	// Set up poll for read with timeout
	pfd := []unix.PollFd{{Fd: int32(fd), Events: unix.POLLIN}}
	timeoutMs := int(timeout.Milliseconds())

	n, err := unix.Poll(pfd, timeoutMs)
	if err != nil {
		if errors.Is(err, unix.EINTR) {
			return 0, nil // Interrupted, try again
		}
		return 0, fmt.Errorf("serial: poll: %w", err)
	}
	if n == 0 {
		return 0, ErrTimeout
	}

	// Check for errors
	if pfd[0].Revents&(unix.POLLERR|unix.POLLHUP|unix.POLLNVAL) != 0 {
		return 0, io.EOF
	}

	// Read available data
	n, err = unix.Read(fd, buf)
	if err != nil {
		return 0, fmt.Errorf("serial: read: %w", err)
	}
	return n, nil
}

// Write writes buf to the port.
// Returns the number of bytes written and any error.
func (p *Port) Write(buf []byte) (int, error) {
	p.mu.Lock()
	if p.closed {
		p.mu.Unlock()
		return 0, ErrClosed
	}
	fd := p.fd
	p.mu.Unlock()

	n, err := unix.Write(fd, buf)
	if err != nil {
		return 0, fmt.Errorf("serial: write: %w", err)
	}
	return n, nil
}

// Close closes the serial port or socket.
func (p *Port) Close() error {
	p.mu.Lock()
	defer p.mu.Unlock()

	if p.closed {
		return nil
	}
	p.closed = true

	// Restore original settings if possible (only for serial ports, not sockets)
	if p.oldTermios != nil && !p.isSocket {
		_ = unix.IoctlSetTermios(p.fd, ioctlSetTermios, p.oldTermios)
	}

	return unix.Close(p.fd)
}

// Device returns the device path.
func (p *Port) Device() string {
	return p.device
}

// Fd returns the underlying file descriptor.
// This is used to pass the fd to chelper serialqueue.
// The caller must not close the fd - use Port.Close() instead.
func (p *Port) Fd() int {
	p.mu.Lock()
	defer p.mu.Unlock()
	return p.fd
}

// SetReadTimeout sets the read timeout.
func (p *Port) SetReadTimeout(d time.Duration) {
	p.mu.Lock()
	p.config.ReadTimeout = d
	p.mu.Unlock()
}

// Flush discards any data in the input and output buffers.
func (p *Port) Flush() error {
	p.mu.Lock()
	if p.closed {
		p.mu.Unlock()
		return ErrClosed
	}
	fd := p.fd
	p.mu.Unlock()

	return unix.IoctlSetInt(fd, ioctlTCFlush, unix.TCIOFLUSH)
}

// setModemControl sets RTS and DTR signals.
// Note: Some USB serial adapters don't support modem control, so errors are logged but not fatal.
func (p *Port) setModemControl(rts, dtr bool) error {
	// On macOS, we need to use pointer-based ioctl for TIOCMGET/TIOCMSET
	var status int32

	// Try to get current modem status
	_, _, errno := unix.Syscall(unix.SYS_IOCTL, uintptr(p.fd), uintptr(unix.TIOCMGET), uintptr(unsafe.Pointer(&status)))
	if errno != 0 {
		// Many USB serial adapters don't support modem control - not fatal
		return nil
	}

	if rts {
		status |= unix.TIOCM_RTS
	} else {
		status &^= unix.TIOCM_RTS
	}
	if dtr {
		status |= unix.TIOCM_DTR
	} else {
		status &^= unix.TIOCM_DTR
	}

	_, _, errno = unix.Syscall(unix.SYS_IOCTL, uintptr(p.fd), uintptr(unix.TIOCMSET), uintptr(unsafe.Pointer(&status)))
	if errno != 0 {
		// Not fatal - some adapters don't support this
		return nil
	}

	return nil
}

// SetRTS sets the RTS signal.
func (p *Port) SetRTS(on bool) error {
	p.mu.Lock()
	defer p.mu.Unlock()
	if p.closed {
		return ErrClosed
	}

	status, err := unix.IoctlGetInt(p.fd, unix.TIOCMGET)
	if err != nil {
		return err
	}
	if on {
		status |= unix.TIOCM_RTS
	} else {
		status &^= unix.TIOCM_RTS
	}
	return unix.IoctlSetInt(p.fd, unix.TIOCMSET, status)
}

// SetDTR sets the DTR signal.
func (p *Port) SetDTR(on bool) error {
	p.mu.Lock()
	defer p.mu.Unlock()
	if p.closed {
		return ErrClosed
	}

	status, err := unix.IoctlGetInt(p.fd, unix.TIOCMGET)
	if err != nil {
		return err
	}
	if on {
		status |= unix.TIOCM_DTR
	} else {
		status &^= unix.TIOCM_DTR
	}
	return unix.IoctlSetInt(p.fd, unix.TIOCMSET, status)
}

// SendBreak sends a break signal.
func (p *Port) SendBreak() error {
	p.mu.Lock()
	if p.closed {
		p.mu.Unlock()
		return ErrClosed
	}
	fd := p.fd
	p.mu.Unlock()

	return unix.IoctlSetInt(fd, ioctlTCSBrk, 0)
}

// setCustomBaudRate sets a custom baud rate on macOS using IOSSIOSPEED.
func setCustomBaudRate(fd int, baud int) error {
	// IOSSIOSPEED is macOS-specific ioctl for setting custom baud rates
	// Value: 0x80045402 (_IOW('T', 2, speed_t))
	const IOSSIOSPEED = 0x80045402
	speed := uint32(baud)
	return unix.IoctlSetPointerInt(fd, IOSSIOSPEED, int(speed))
}

// baudRateToSpeed converts a baud rate to a speed constant.
// Returns (speed, customBaud, error) where customBaud > 0 means use IOSSIOSPEED on macOS.
func baudRateToSpeed(baud int) (uint32, int, error) {
	speeds := map[int]uint32{
		50:      unix.B50,
		75:      unix.B75,
		110:     unix.B110,
		134:     unix.B134,
		150:     unix.B150,
		200:     unix.B200,
		300:     unix.B300,
		600:     unix.B600,
		1200:    unix.B1200,
		1800:    unix.B1800,
		2400:    unix.B2400,
		4800:    unix.B4800,
		9600:    unix.B9600,
		19200:   unix.B19200,
		38400:   unix.B38400,
		57600:   unix.B57600,
		115200:  unix.B115200,
		230400:  unix.B230400,
	}

	// Handle platform-specific high baud rates
	if runtime.GOOS == "linux" {
		speeds[460800] = 0x1004  // B460800
		speeds[500000] = 0x1005  // B500000
		speeds[576000] = 0x1006  // B576000
		speeds[921600] = 0x1007  // B921600
		speeds[1000000] = 0x1008 // B1000000
		speeds[1152000] = 0x1009 // B1152000
		speeds[1500000] = 0x100A // B1500000
		speeds[2000000] = 0x100B // B2000000
		speeds[2500000] = 0x100C // B2500000
		speeds[3000000] = 0x100D // B3000000
		speeds[3500000] = 0x100E // B3500000
		speeds[4000000] = 0x100F // B4000000
		// Klipper default
		speeds[250000] = 0x1003 // B250000 (custom rate)
	}

	if speed, ok := speeds[baud]; ok {
		return speed, 0, nil
	}

	// For non-standard baud rates on Linux, we can try to set it directly
	if runtime.GOOS == "linux" {
		// Use BOTHER to set arbitrary baud rate
		return 0x1000 | uint32(baud), 0, nil // BOTHER
	}

	// For macOS, use a standard rate then set custom via IOSSIOSPEED
	if runtime.GOOS == "darwin" {
		// Use 9600 as base, then set custom baud rate
		return unix.B9600, baud, nil
	}

	return 0, 0, fmt.Errorf("serial: unsupported baud rate %d", baud)
}

// Detect attempts to detect and open an MCU on any available port.
func Detect(cfg Config, timeout time.Duration) (*Port, error) {
	ports, err := ListPorts()
	if err != nil {
		return nil, err
	}

	if len(ports) == 0 {
		return nil, errors.New("serial: no serial ports found")
	}

	deadline := time.Now().Add(timeout)
	for time.Now().Before(deadline) {
		for _, device := range ports {
			cfg.Device = device
			port, err := Open(cfg)
			if err != nil {
				continue
			}
			return port, nil
		}
		time.Sleep(500 * time.Millisecond)
	}

	return nil, fmt.Errorf("serial: no MCU found on ports %v", ports)
}

// IsDeviceAvailable checks if a device path exists and is accessible.
func IsDeviceAvailable(device string) bool {
	info, err := os.Stat(device)
	if err != nil {
		return false
	}
	// Check if it's a character device
	if info.Mode()&os.ModeCharDevice == 0 {
		return false
	}
	// Check if we can open it
	fd, err := unix.Open(device, unix.O_RDWR|unix.O_NOCTTY|unix.O_NONBLOCK, 0)
	if err != nil {
		return false
	}
	unix.Close(fd)
	return true
}

// ResolveDevice resolves a device path, following symlinks.
func ResolveDevice(device string) (string, error) {
	// Handle by-id and by-path symlinks
	if strings.HasPrefix(device, "/dev/serial/") {
		resolved, err := filepath.EvalSymlinks(device)
		if err != nil {
			return "", fmt.Errorf("serial: resolve %s: %w", device, err)
		}
		return resolved, nil
	}
	return device, nil
}
