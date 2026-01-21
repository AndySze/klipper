// Log file rotation support for Klipper Go migration
//
// Provides automatic log file rotation based on size and count.
//
// Copyright (C) 2026  Go Migration Team
//
// This file may be distributed under the terms of the GNU GPLv3 license.

package log

import (
	"compress/gzip"
	"fmt"
	"io"
	"os"
	"path/filepath"
	"sort"
	"strconv"
	"strings"
	"sync"
	"time"
)

// RotatingFileWriter implements io.Writer with automatic file rotation.
type RotatingFileWriter struct {
	mu          sync.Mutex
	filename    string
	maxSize     int64 // Maximum file size in bytes before rotation
	maxBackups  int   // Maximum number of backup files to keep
	compress    bool  // Whether to compress rotated files
	currentSize int64
	file        *os.File
}

// RotationConfig configures log file rotation.
type RotationConfig struct {
	// Filename is the path to the log file.
	Filename string

	// MaxSize is the maximum size in megabytes before rotation.
	// Default is 10 MB.
	MaxSize int

	// MaxBackups is the maximum number of old log files to retain.
	// Default is 5.
	MaxBackups int

	// Compress determines if rotated files should be gzipped.
	// Default is false.
	Compress bool
}

// NewRotatingFileWriter creates a new rotating file writer.
func NewRotatingFileWriter(config RotationConfig) (*RotatingFileWriter, error) {
	if config.Filename == "" {
		return nil, fmt.Errorf("filename is required")
	}

	maxSize := config.MaxSize
	if maxSize <= 0 {
		maxSize = 10 // 10 MB default
	}

	maxBackups := config.MaxBackups
	if maxBackups <= 0 {
		maxBackups = 5
	}

	w := &RotatingFileWriter{
		filename:   config.Filename,
		maxSize:    int64(maxSize) * 1024 * 1024,
		maxBackups: maxBackups,
		compress:   config.Compress,
	}

	if err := w.openFile(); err != nil {
		return nil, err
	}

	return w, nil
}

// openFile opens the log file for writing.
func (w *RotatingFileWriter) openFile() error {
	// Ensure directory exists
	dir := filepath.Dir(w.filename)
	if err := os.MkdirAll(dir, 0755); err != nil {
		return fmt.Errorf("create log directory: %w", err)
	}

	// Open file in append mode
	f, err := os.OpenFile(w.filename, os.O_APPEND|os.O_CREATE|os.O_WRONLY, 0644)
	if err != nil {
		return fmt.Errorf("open log file: %w", err)
	}

	// Get current file size
	info, err := f.Stat()
	if err != nil {
		f.Close()
		return fmt.Errorf("stat log file: %w", err)
	}

	w.file = f
	w.currentSize = info.Size()
	return nil
}

// Write implements io.Writer.
func (w *RotatingFileWriter) Write(p []byte) (n int, err error) {
	w.mu.Lock()
	defer w.mu.Unlock()

	// Check if rotation is needed
	if w.currentSize+int64(len(p)) > w.maxSize {
		if err := w.rotate(); err != nil {
			return 0, fmt.Errorf("rotate log file: %w", err)
		}
	}

	n, err = w.file.Write(p)
	w.currentSize += int64(n)
	return n, err
}

// rotate performs log file rotation.
func (w *RotatingFileWriter) rotate() error {
	// Close current file
	if err := w.file.Close(); err != nil {
		return fmt.Errorf("close current file: %w", err)
	}

	// Generate rotated filename with timestamp
	timestamp := time.Now().Format("20060102-150405")
	ext := filepath.Ext(w.filename)
	base := strings.TrimSuffix(w.filename, ext)
	rotatedName := fmt.Sprintf("%s.%s%s", base, timestamp, ext)

	// Rename current file to rotated name
	if err := os.Rename(w.filename, rotatedName); err != nil {
		// Try to reopen the original file
		w.openFile()
		return fmt.Errorf("rename log file: %w", err)
	}

	// Compress if enabled
	if w.compress {
		go w.compressFile(rotatedName)
	}

	// Clean up old backups
	go w.cleanOldBackups()

	// Open new file
	return w.openFile()
}

// compressFile compresses a rotated log file.
func (w *RotatingFileWriter) compressFile(filename string) {
	src, err := os.Open(filename)
	if err != nil {
		return
	}
	defer src.Close()

	dst, err := os.Create(filename + ".gz")
	if err != nil {
		return
	}

	gz := gzip.NewWriter(dst)
	if _, err := io.Copy(gz, src); err != nil {
		gz.Close()
		dst.Close()
		os.Remove(filename + ".gz")
		return
	}

	gz.Close()
	dst.Close()
	src.Close()
	os.Remove(filename)
}

// cleanOldBackups removes old backup files exceeding maxBackups.
func (w *RotatingFileWriter) cleanOldBackups() {
	dir := filepath.Dir(w.filename)
	base := filepath.Base(w.filename)
	ext := filepath.Ext(base)
	prefix := strings.TrimSuffix(base, ext)

	// Find all backup files
	entries, err := os.ReadDir(dir)
	if err != nil {
		return
	}

	var backups []string
	for _, entry := range entries {
		name := entry.Name()
		if strings.HasPrefix(name, prefix+".") && name != base {
			// Check if it's a rotated file (has timestamp pattern)
			if isRotatedFile(name, prefix, ext) {
				backups = append(backups, filepath.Join(dir, name))
			}
		}
	}

	// Sort by modification time (oldest first)
	sort.Slice(backups, func(i, j int) bool {
		iInfo, _ := os.Stat(backups[i])
		jInfo, _ := os.Stat(backups[j])
		if iInfo == nil || jInfo == nil {
			return false
		}
		return iInfo.ModTime().Before(jInfo.ModTime())
	})

	// Remove oldest files if we have too many
	for len(backups) > w.maxBackups {
		os.Remove(backups[0])
		backups = backups[1:]
	}
}

// isRotatedFile checks if a filename matches the rotation pattern.
func isRotatedFile(name, prefix, ext string) bool {
	// Pattern: prefix.YYYYMMDD-HHMMSS.ext or prefix.YYYYMMDD-HHMMSS.ext.gz
	name = strings.TrimSuffix(name, ".gz")
	name = strings.TrimSuffix(name, ext)
	name = strings.TrimPrefix(name, prefix+".")

	// Check if remaining part is a timestamp
	if len(name) != 15 { // YYYYMMDD-HHMMSS
		return false
	}
	if name[8] != '-' {
		return false
	}

	// Verify numeric parts
	_, err1 := strconv.Atoi(name[:8])
	_, err2 := strconv.Atoi(name[9:])
	return err1 == nil && err2 == nil
}

// Close closes the rotating file writer.
func (w *RotatingFileWriter) Close() error {
	w.mu.Lock()
	defer w.mu.Unlock()

	if w.file != nil {
		return w.file.Close()
	}
	return nil
}

// Sync syncs the underlying file.
func (w *RotatingFileWriter) Sync() error {
	w.mu.Lock()
	defer w.mu.Unlock()

	if w.file != nil {
		return w.file.Sync()
	}
	return nil
}

// CurrentSize returns the current file size.
func (w *RotatingFileWriter) CurrentSize() int64 {
	w.mu.Lock()
	defer w.mu.Unlock()
	return w.currentSize
}

// Filename returns the current log filename.
func (w *RotatingFileWriter) Filename() string {
	return w.filename
}

// NewFileLogger creates a logger that writes to a rotating file.
func NewFileLogger(prefix string, config RotationConfig) (*Logger, *RotatingFileWriter, error) {
	writer, err := NewRotatingFileWriter(config)
	if err != nil {
		return nil, nil, err
	}

	logger := New(prefix)
	logger.SetWriter(writer)
	logger.SetColorize(false) // No colors in file output

	return logger, writer, nil
}

// MultiWriter writes to multiple writers simultaneously.
type MultiWriter struct {
	writers []io.Writer
}

// NewMultiWriter creates a writer that writes to all provided writers.
func NewMultiWriter(writers ...io.Writer) *MultiWriter {
	return &MultiWriter{writers: writers}
}

// Write implements io.Writer.
func (mw *MultiWriter) Write(p []byte) (n int, err error) {
	for _, w := range mw.writers {
		n, err = w.Write(p)
		if err != nil {
			return n, err
		}
	}
	return len(p), nil
}

// NewConsoleAndFileLogger creates a logger that writes to both console and file.
func NewConsoleAndFileLogger(prefix string, config RotationConfig) (*Logger, *RotatingFileWriter, error) {
	fileWriter, err := NewRotatingFileWriter(config)
	if err != nil {
		return nil, nil, err
	}

	// Console logger for colorized output
	consoleLogger := New(prefix)

	// File logger for non-colorized output to file
	multiWriter := NewMultiWriter(os.Stderr, fileWriter)

	logger := New(prefix)
	logger.SetWriter(multiWriter)
	logger.SetColorize(false) // Colors don't work well with file output

	// Return the console logger's settings for better UX
	logger.SetColorize(consoleLogger.colorize)

	return logger, fileWriter, nil
}
