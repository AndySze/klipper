//go:build linux

package serial

import "golang.org/x/sys/unix"

// setSpeed sets the baud rate on the termios struct for Linux.
func setSpeed(termios *unix.Termios, speed uint32) {
	termios.Ispeed = speed
	termios.Ospeed = speed
}
