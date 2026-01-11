//go:build linux

package serial

import "golang.org/x/sys/unix"

// Platform-specific ioctl constants for Linux
const (
	ioctlGetTermios = unix.TCGETS
	ioctlSetTermios = unix.TCSETS
	ioctlTCFlush    = unix.TCFLSH
	ioctlTCSBrk     = unix.TCSBRK
)
