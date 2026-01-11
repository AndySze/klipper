//go:build darwin

package serial

import "golang.org/x/sys/unix"

// setSpeed sets the baud rate on the termios struct for macOS.
func setSpeed(termios *unix.Termios, speed uint32) {
	termios.Ispeed = uint64(speed)
	termios.Ospeed = uint64(speed)
}
