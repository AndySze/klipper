//go:build darwin

package serial

import "golang.org/x/sys/unix"

// Platform-specific ioctl constants for macOS
const (
	ioctlGetTermios = unix.TIOCGETA
	ioctlSetTermios = unix.TIOCSETA
	ioctlTCFlush    = unix.TIOCFLUSH
	ioctlTCSBrk     = unix.TIOCSBRK
)
