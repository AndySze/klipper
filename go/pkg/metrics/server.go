// HTTP server for Prometheus metrics endpoint
//
// Provides an HTTP endpoint at /metrics for Prometheus scraping.
// Configurable listen address and optional basic authentication.
//
// Example usage:
//
//	server := metrics.NewMetricsServer(metrics.GlobalMetrics(), ":9100")
//	go server.Start()
//	defer server.Shutdown(context.Background())
//
// Then configure Prometheus to scrape http://host:9100/metrics
//
// Copyright (C) 2026 Go Migration Team
//
// This file may be distributed under the terms of the GNU GPLv3 license.

package metrics

import (
	"context"
	"crypto/subtle"
	"fmt"
	"net/http"
	"sync"
	"time"
)

// MetricsServer serves Prometheus metrics over HTTP
type MetricsServer struct {
	km       *KlipperMetrics
	addr     string
	server   *http.Server
	mux      *http.ServeMux

	// Optional basic auth
	username string
	password string

	// State
	mu        sync.RWMutex
	running   bool
	startTime time.Time
}

// MetricsServerConfig holds server configuration
type MetricsServerConfig struct {
	// Address to listen on (e.g., ":9100" or "127.0.0.1:9100")
	Address string

	// Optional basic auth credentials
	Username string
	Password string

	// Read timeout for HTTP requests
	ReadTimeout time.Duration

	// Write timeout for HTTP responses
	WriteTimeout time.Duration
}

// DefaultMetricsServerConfig returns default server configuration
func DefaultMetricsServerConfig() MetricsServerConfig {
	return MetricsServerConfig{
		Address:      ":9100",
		ReadTimeout:  10 * time.Second,
		WriteTimeout: 10 * time.Second,
	}
}

// NewMetricsServer creates a new metrics server with default config
func NewMetricsServer(km *KlipperMetrics, addr string) *MetricsServer {
	config := DefaultMetricsServerConfig()
	config.Address = addr
	return NewMetricsServerWithConfig(km, config)
}

// NewMetricsServerWithConfig creates a new metrics server with custom config
func NewMetricsServerWithConfig(km *KlipperMetrics, config MetricsServerConfig) *MetricsServer {
	ms := &MetricsServer{
		km:       km,
		addr:     config.Address,
		mux:      http.NewServeMux(),
		username: config.Username,
		password: config.Password,
	}

	// Register handlers
	ms.mux.HandleFunc("/metrics", ms.handleMetrics)
	ms.mux.HandleFunc("/health", ms.handleHealth)
	ms.mux.HandleFunc("/ready", ms.handleReady)
	ms.mux.HandleFunc("/", ms.handleRoot)

	ms.server = &http.Server{
		Addr:         config.Address,
		Handler:      ms.mux,
		ReadTimeout:  config.ReadTimeout,
		WriteTimeout: config.WriteTimeout,
	}

	return ms
}

// Start starts the metrics server (blocks until server stops)
func (ms *MetricsServer) Start() error {
	ms.mu.Lock()
	ms.running = true
	ms.startTime = time.Now()
	ms.mu.Unlock()

	err := ms.server.ListenAndServe()
	if err != nil && err != http.ErrServerClosed {
		return fmt.Errorf("metrics server error: %w", err)
	}
	return nil
}

// StartAsync starts the metrics server in a goroutine
func (ms *MetricsServer) StartAsync() chan error {
	errCh := make(chan error, 1)
	go func() {
		if err := ms.Start(); err != nil {
			errCh <- err
		}
		close(errCh)
	}()
	return errCh
}

// Shutdown gracefully shuts down the server
func (ms *MetricsServer) Shutdown(ctx context.Context) error {
	ms.mu.Lock()
	ms.running = false
	ms.mu.Unlock()

	return ms.server.Shutdown(ctx)
}

// IsRunning returns whether the server is running
func (ms *MetricsServer) IsRunning() bool {
	ms.mu.RLock()
	defer ms.mu.RUnlock()
	return ms.running
}

// GetAddress returns the server address
func (ms *MetricsServer) GetAddress() string {
	return ms.addr
}

// handleMetrics serves Prometheus metrics
func (ms *MetricsServer) handleMetrics(w http.ResponseWriter, r *http.Request) {
	// Check auth if configured
	if !ms.checkAuth(w, r) {
		return
	}

	// Only allow GET and HEAD
	if r.Method != http.MethodGet && r.Method != http.MethodHead {
		http.Error(w, "Method not allowed", http.StatusMethodNotAllowed)
		return
	}

	// Set content type for Prometheus
	w.Header().Set("Content-Type", "text/plain; version=0.0.4; charset=utf-8")

	// Gather and write metrics
	output := ms.km.Gather()

	if r.Method == http.MethodHead {
		w.Header().Set("Content-Length", fmt.Sprintf("%d", len(output)))
		return
	}

	_, _ = w.Write([]byte(output))
}

// handleHealth provides a health check endpoint
func (ms *MetricsServer) handleHealth(w http.ResponseWriter, r *http.Request) {
	w.Header().Set("Content-Type", "text/plain")
	w.WriteHeader(http.StatusOK)
	_, _ = w.Write([]byte("OK\n"))
}

// handleReady provides a readiness check endpoint
func (ms *MetricsServer) handleReady(w http.ResponseWriter, r *http.Request) {
	ms.mu.RLock()
	running := ms.running
	ms.mu.RUnlock()

	w.Header().Set("Content-Type", "text/plain")
	if running {
		w.WriteHeader(http.StatusOK)
		_, _ = w.Write([]byte("Ready\n"))
	} else {
		w.WriteHeader(http.StatusServiceUnavailable)
		_, _ = w.Write([]byte("Not Ready\n"))
	}
}

// handleRoot provides a simple landing page
func (ms *MetricsServer) handleRoot(w http.ResponseWriter, r *http.Request) {
	if r.URL.Path != "/" {
		http.NotFound(w, r)
		return
	}

	w.Header().Set("Content-Type", "text/html; charset=utf-8")
	html := `<!DOCTYPE html>
<html>
<head>
<title>Klipper Metrics</title>
<style>
body { font-family: sans-serif; margin: 40px; }
h1 { color: #333; }
a { color: #0066cc; }
.endpoint { margin: 10px 0; }
</style>
</head>
<body>
<h1>Klipper Go Host Metrics</h1>
<p>This server provides Prometheus-compatible metrics for monitoring.</p>
<div class="endpoint"><a href="/metrics">/metrics</a> - Prometheus metrics endpoint</div>
<div class="endpoint"><a href="/health">/health</a> - Health check</div>
<div class="endpoint"><a href="/ready">/ready</a> - Readiness check</div>
</body>
</html>`
	_, _ = w.Write([]byte(html))
}

// checkAuth verifies basic auth if configured
func (ms *MetricsServer) checkAuth(w http.ResponseWriter, r *http.Request) bool {
	// Skip auth check if not configured
	if ms.username == "" && ms.password == "" {
		return true
	}

	username, password, ok := r.BasicAuth()
	if !ok {
		ms.unauthorizedResponse(w)
		return false
	}

	// Use constant-time comparison to prevent timing attacks
	usernameMatch := subtle.ConstantTimeCompare([]byte(username), []byte(ms.username)) == 1
	passwordMatch := subtle.ConstantTimeCompare([]byte(password), []byte(ms.password)) == 1

	if !usernameMatch || !passwordMatch {
		ms.unauthorizedResponse(w)
		return false
	}

	return true
}

// unauthorizedResponse sends a 401 response
func (ms *MetricsServer) unauthorizedResponse(w http.ResponseWriter) {
	w.Header().Set("WWW-Authenticate", `Basic realm="Klipper Metrics"`)
	http.Error(w, "Unauthorized", http.StatusUnauthorized)
}

// GetStatus returns server status for diagnostics
func (ms *MetricsServer) GetStatus() map[string]any {
	ms.mu.RLock()
	defer ms.mu.RUnlock()

	status := map[string]any{
		"address": ms.addr,
		"running": ms.running,
	}

	if ms.running {
		status["uptime"] = time.Since(ms.startTime).Seconds()
	}

	return status
}
