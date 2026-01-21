// Unit tests for metrics HTTP server
//
// Copyright (C) 2026 Go Migration Team
//
// This file may be distributed under the terms of the GNU GPLv3 license.

package metrics

import (
	"context"
	"io"
	"net/http"
	"net/http/httptest"
	"strings"
	"testing"
	"time"
)

// TestMetricsServerBasic tests basic server creation
func TestMetricsServerBasic(t *testing.T) {
	km := NewKlipperMetrics()
	server := NewMetricsServer(km, ":0")

	if server == nil {
		t.Fatal("server should not be nil")
	}

	if !strings.Contains(server.GetAddress(), ":") {
		t.Error("address should contain port")
	}

	if server.IsRunning() {
		t.Error("server should not be running before Start")
	}
}

// TestMetricsServerConfig tests server configuration
func TestMetricsServerConfig(t *testing.T) {
	km := NewKlipperMetrics()
	config := MetricsServerConfig{
		Address:      ":9200",
		Username:     "admin",
		Password:     "secret",
		ReadTimeout:  5 * time.Second,
		WriteTimeout: 5 * time.Second,
	}

	server := NewMetricsServerWithConfig(km, config)

	if server.GetAddress() != ":9200" {
		t.Errorf("expected address :9200, got %s", server.GetAddress())
	}
}

// TestDefaultConfig tests default configuration
func TestDefaultConfig(t *testing.T) {
	config := DefaultMetricsServerConfig()

	if config.Address != ":9100" {
		t.Errorf("expected default address :9100, got %s", config.Address)
	}
	if config.ReadTimeout != 10*time.Second {
		t.Error("unexpected read timeout")
	}
	if config.WriteTimeout != 10*time.Second {
		t.Error("unexpected write timeout")
	}
}

// TestHandleMetrics tests the /metrics endpoint
func TestHandleMetrics(t *testing.T) {
	km := NewKlipperMetrics()
	km.SetToolheadPosition(100, 200, 10, 50)
	km.SetHeaterStatus("extruder", 200, 210, 0.8)

	server := NewMetricsServer(km, ":0")

	// Create test request
	req := httptest.NewRequest(http.MethodGet, "/metrics", nil)
	w := httptest.NewRecorder()

	server.mux.ServeHTTP(w, req)

	resp := w.Result()
	body, _ := io.ReadAll(resp.Body)

	if resp.StatusCode != http.StatusOK {
		t.Errorf("expected status 200, got %d", resp.StatusCode)
	}

	// Check content type
	contentType := resp.Header.Get("Content-Type")
	if !strings.Contains(contentType, "text/plain") {
		t.Errorf("unexpected content type: %s", contentType)
	}

	// Check metrics content
	bodyStr := string(body)
	if !strings.Contains(bodyStr, "klipper_toolhead_position_mm") {
		t.Error("missing toolhead position metric")
	}
	if !strings.Contains(bodyStr, "klipper_sensor_temperature_celsius") {
		t.Error("missing temperature metric")
	}
}

// TestHandleMetricsHead tests HEAD request to /metrics
func TestHandleMetricsHead(t *testing.T) {
	km := NewKlipperMetrics()
	server := NewMetricsServer(km, ":0")

	req := httptest.NewRequest(http.MethodHead, "/metrics", nil)
	w := httptest.NewRecorder()

	server.mux.ServeHTTP(w, req)

	resp := w.Result()

	if resp.StatusCode != http.StatusOK {
		t.Errorf("expected status 200, got %d", resp.StatusCode)
	}

	// Body should be empty for HEAD
	body, _ := io.ReadAll(resp.Body)
	if len(body) != 0 {
		t.Error("HEAD response should have empty body")
	}
}

// TestHandleMetricsMethodNotAllowed tests unsupported methods
func TestHandleMetricsMethodNotAllowed(t *testing.T) {
	km := NewKlipperMetrics()
	server := NewMetricsServer(km, ":0")

	req := httptest.NewRequest(http.MethodPost, "/metrics", nil)
	w := httptest.NewRecorder()

	server.mux.ServeHTTP(w, req)

	resp := w.Result()

	if resp.StatusCode != http.StatusMethodNotAllowed {
		t.Errorf("expected status 405, got %d", resp.StatusCode)
	}
}

// TestHandleHealth tests the /health endpoint
func TestHandleHealth(t *testing.T) {
	km := NewKlipperMetrics()
	server := NewMetricsServer(km, ":0")

	req := httptest.NewRequest(http.MethodGet, "/health", nil)
	w := httptest.NewRecorder()

	server.mux.ServeHTTP(w, req)

	resp := w.Result()
	body, _ := io.ReadAll(resp.Body)

	if resp.StatusCode != http.StatusOK {
		t.Errorf("expected status 200, got %d", resp.StatusCode)
	}

	if !strings.Contains(string(body), "OK") {
		t.Error("health check should return OK")
	}
}

// TestHandleReady tests the /ready endpoint
func TestHandleReady(t *testing.T) {
	km := NewKlipperMetrics()
	server := NewMetricsServer(km, ":0")

	// Before starting, should return ready (running flag set to false but handler works)
	req := httptest.NewRequest(http.MethodGet, "/ready", nil)
	w := httptest.NewRecorder()

	server.mux.ServeHTTP(w, req)

	resp := w.Result()

	// When not running, should return service unavailable
	if resp.StatusCode != http.StatusServiceUnavailable {
		t.Errorf("expected status 503 when not running, got %d", resp.StatusCode)
	}

	// Simulate running state
	server.mu.Lock()
	server.running = true
	server.mu.Unlock()

	w = httptest.NewRecorder()
	server.mux.ServeHTTP(w, req)

	resp = w.Result()
	if resp.StatusCode != http.StatusOK {
		t.Errorf("expected status 200 when running, got %d", resp.StatusCode)
	}
}

// TestHandleRoot tests the root landing page
func TestHandleRoot(t *testing.T) {
	km := NewKlipperMetrics()
	server := NewMetricsServer(km, ":0")

	req := httptest.NewRequest(http.MethodGet, "/", nil)
	w := httptest.NewRecorder()

	server.mux.ServeHTTP(w, req)

	resp := w.Result()
	body, _ := io.ReadAll(resp.Body)

	if resp.StatusCode != http.StatusOK {
		t.Errorf("expected status 200, got %d", resp.StatusCode)
	}

	// Should contain HTML with links
	bodyStr := string(body)
	if !strings.Contains(bodyStr, "<html>") {
		t.Error("root should return HTML")
	}
	if !strings.Contains(bodyStr, "/metrics") {
		t.Error("root should link to /metrics")
	}
	if !strings.Contains(bodyStr, "/health") {
		t.Error("root should link to /health")
	}
}

// TestHandleRootNotFound tests 404 for unknown paths
func TestHandleRootNotFound(t *testing.T) {
	km := NewKlipperMetrics()
	server := NewMetricsServer(km, ":0")

	req := httptest.NewRequest(http.MethodGet, "/unknown", nil)
	w := httptest.NewRecorder()

	server.mux.ServeHTTP(w, req)

	resp := w.Result()

	if resp.StatusCode != http.StatusNotFound {
		t.Errorf("expected status 404, got %d", resp.StatusCode)
	}
}

// TestBasicAuth tests basic authentication
func TestBasicAuth(t *testing.T) {
	km := NewKlipperMetrics()
	config := MetricsServerConfig{
		Address:      ":0",
		Username:     "admin",
		Password:     "secret123",
		ReadTimeout:  10 * time.Second,
		WriteTimeout: 10 * time.Second,
	}
	server := NewMetricsServerWithConfig(km, config)

	// Request without auth
	req := httptest.NewRequest(http.MethodGet, "/metrics", nil)
	w := httptest.NewRecorder()
	server.mux.ServeHTTP(w, req)

	resp := w.Result()
	if resp.StatusCode != http.StatusUnauthorized {
		t.Errorf("expected 401 without auth, got %d", resp.StatusCode)
	}

	// Check WWW-Authenticate header
	if resp.Header.Get("WWW-Authenticate") == "" {
		t.Error("should set WWW-Authenticate header")
	}

	// Request with wrong credentials
	req = httptest.NewRequest(http.MethodGet, "/metrics", nil)
	req.SetBasicAuth("admin", "wrongpassword")
	w = httptest.NewRecorder()
	server.mux.ServeHTTP(w, req)

	resp = w.Result()
	if resp.StatusCode != http.StatusUnauthorized {
		t.Errorf("expected 401 with wrong password, got %d", resp.StatusCode)
	}

	// Request with correct credentials
	req = httptest.NewRequest(http.MethodGet, "/metrics", nil)
	req.SetBasicAuth("admin", "secret123")
	w = httptest.NewRecorder()
	server.mux.ServeHTTP(w, req)

	resp = w.Result()
	if resp.StatusCode != http.StatusOK {
		t.Errorf("expected 200 with correct auth, got %d", resp.StatusCode)
	}
}

// TestNoAuthWhenNotConfigured tests that auth is skipped when not configured
func TestNoAuthWhenNotConfigured(t *testing.T) {
	km := NewKlipperMetrics()
	server := NewMetricsServer(km, ":0")

	req := httptest.NewRequest(http.MethodGet, "/metrics", nil)
	w := httptest.NewRecorder()
	server.mux.ServeHTTP(w, req)

	resp := w.Result()
	if resp.StatusCode != http.StatusOK {
		t.Errorf("expected 200 without auth config, got %d", resp.StatusCode)
	}
}

// TestGetStatus tests server status
func TestGetStatus(t *testing.T) {
	km := NewKlipperMetrics()
	server := NewMetricsServer(km, ":9100")

	status := server.GetStatus()

	if status["address"] != ":9100" {
		t.Error("status should include address")
	}
	if status["running"].(bool) {
		t.Error("should not be running")
	}

	// Simulate running
	server.mu.Lock()
	server.running = true
	server.startTime = time.Now().Add(-10 * time.Second)
	server.mu.Unlock()

	status = server.GetStatus()
	if !status["running"].(bool) {
		t.Error("should be running")
	}
	if uptime, ok := status["uptime"].(float64); !ok || uptime < 9 {
		t.Error("uptime should be tracked")
	}
}

// TestShutdown tests graceful shutdown
func TestShutdown(t *testing.T) {
	km := NewKlipperMetrics()
	server := NewMetricsServer(km, ":0")

	// Start server in background
	errCh := server.StartAsync()

	// Give it time to start
	time.Sleep(50 * time.Millisecond)

	if !server.IsRunning() {
		t.Error("server should be running after StartAsync")
	}

	// Shutdown
	ctx, cancel := context.WithTimeout(context.Background(), 5*time.Second)
	defer cancel()

	if err := server.Shutdown(ctx); err != nil {
		t.Errorf("shutdown failed: %v", err)
	}

	if server.IsRunning() {
		t.Error("server should not be running after Shutdown")
	}

	// Check for any errors
	select {
	case err := <-errCh:
		if err != nil {
			t.Errorf("server error: %v", err)
		}
	case <-time.After(1 * time.Second):
		// OK, no error
	}
}

// BenchmarkHandleMetrics benchmarks the metrics endpoint
func BenchmarkHandleMetrics(b *testing.B) {
	km := NewKlipperMetrics()
	km.SetToolheadPosition(100, 200, 10, 50)
	km.SetHeaterStatus("extruder", 200, 210, 0.8)
	km.SetMCUStatus("mcu", true, 0, 72000000)

	server := NewMetricsServer(km, ":0")

	req := httptest.NewRequest(http.MethodGet, "/metrics", nil)

	b.ResetTimer()
	for i := 0; i < b.N; i++ {
		w := httptest.NewRecorder()
		server.mux.ServeHTTP(w, req)
	}
}
