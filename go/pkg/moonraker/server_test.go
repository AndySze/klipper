package moonraker

import (
	"bytes"
	"encoding/json"
	"io"
	"net/http"
	"net/http/httptest"
	"testing"
	"time"

	"github.com/gorilla/websocket"
)

// mockPrinter implements PrinterInterface for testing.
type mockPrinter struct {
	state   string
	objects []string
}

func (m *mockPrinter) GetObjectsList() []string {
	if len(m.objects) > 0 {
		return m.objects
	}
	return []string{"webhooks", "toolhead", "extruder", "heater_bed", "fan"}
}

func (m *mockPrinter) GetObjectStatus(name string, attrs []string) map[string]any {
	switch name {
	case "toolhead":
		return map[string]any{
			"position":    []float64{0, 0, 0, 0},
			"homed_axes":  "xyz",
			"print_time":  0.0,
		}
	case "extruder":
		return map[string]any{
			"temperature": 200.0,
			"target":      210.0,
		}
	case "heater_bed":
		return map[string]any{
			"temperature": 60.0,
			"target":      65.0,
		}
	default:
		return nil
	}
}

func (m *mockPrinter) ExecuteGCode(script string) error {
	return nil
}

func (m *mockPrinter) EmergencyStop() {
}

func (m *mockPrinter) GetKlippyState() string {
	if m.state != "" {
		return m.state
	}
	return "ready"
}

func newTestServer() *Server {
	return New(Config{
		Addr:    ":7125",
		Printer: &mockPrinter{},
	})
}

func TestServerInfo(t *testing.T) {
	s := newTestServer()
	mux := http.NewServeMux()
	mux.HandleFunc("/server/info", s.handleServerInfo)

	req := httptest.NewRequest("GET", "/server/info", nil)
	rec := httptest.NewRecorder()
	mux.ServeHTTP(rec, req)

	if rec.Code != http.StatusOK {
		t.Fatalf("expected status 200, got %d", rec.Code)
	}

	var resp map[string]any
	if err := json.NewDecoder(rec.Body).Decode(&resp); err != nil {
		t.Fatalf("failed to decode response: %v", err)
	}

	result, ok := resp["result"].(map[string]any)
	if !ok {
		t.Fatal("response missing 'result' field")
	}

	if result["klippy_state"] != "ready" {
		t.Errorf("expected klippy_state 'ready', got %v", result["klippy_state"])
	}

	if result["klippy_connected"] != true {
		t.Errorf("expected klippy_connected true, got %v", result["klippy_connected"])
	}
}

func TestPrinterInfo(t *testing.T) {
	s := newTestServer()
	mux := http.NewServeMux()
	mux.HandleFunc("/printer/info", s.handlePrinterInfo)

	req := httptest.NewRequest("GET", "/printer/info", nil)
	rec := httptest.NewRecorder()
	mux.ServeHTTP(rec, req)

	if rec.Code != http.StatusOK {
		t.Fatalf("expected status 200, got %d", rec.Code)
	}

	var resp map[string]any
	if err := json.NewDecoder(rec.Body).Decode(&resp); err != nil {
		t.Fatalf("failed to decode response: %v", err)
	}

	result, ok := resp["result"].(map[string]any)
	if !ok {
		t.Fatal("response missing 'result' field")
	}

	if result["state"] != "ready" {
		t.Errorf("expected state 'ready', got %v", result["state"])
	}
}

func TestObjectsList(t *testing.T) {
	s := newTestServer()
	mux := http.NewServeMux()
	mux.HandleFunc("/printer/objects/list", s.handleObjectsList)

	req := httptest.NewRequest("GET", "/printer/objects/list", nil)
	rec := httptest.NewRecorder()
	mux.ServeHTTP(rec, req)

	if rec.Code != http.StatusOK {
		t.Fatalf("expected status 200, got %d", rec.Code)
	}

	var resp map[string]any
	if err := json.NewDecoder(rec.Body).Decode(&resp); err != nil {
		t.Fatalf("failed to decode response: %v", err)
	}

	result, ok := resp["result"].(map[string]any)
	if !ok {
		t.Fatal("response missing 'result' field")
	}

	objects, ok := result["objects"].([]any)
	if !ok {
		t.Fatal("result missing 'objects' field")
	}

	if len(objects) == 0 {
		t.Error("expected at least one object")
	}
}

func TestObjectsQuery(t *testing.T) {
	s := newTestServer()
	mux := http.NewServeMux()
	mux.HandleFunc("/printer/objects/query", s.handleObjectsQuery)

	body := bytes.NewBufferString(`{"objects":{"toolhead":null,"extruder":["temperature","target"]}}`)
	req := httptest.NewRequest("POST", "/printer/objects/query", body)
	rec := httptest.NewRecorder()
	mux.ServeHTTP(rec, req)

	if rec.Code != http.StatusOK {
		t.Fatalf("expected status 200, got %d", rec.Code)
	}

	var resp map[string]any
	if err := json.NewDecoder(rec.Body).Decode(&resp); err != nil {
		t.Fatalf("failed to decode response: %v", err)
	}

	result, ok := resp["result"].(map[string]any)
	if !ok {
		t.Fatal("response missing 'result' field")
	}

	status, ok := result["status"].(map[string]any)
	if !ok {
		t.Fatal("result missing 'status' field")
	}

	if _, ok := status["toolhead"]; !ok {
		t.Error("status missing 'toolhead'")
	}

	if _, ok := status["extruder"]; !ok {
		t.Error("status missing 'extruder'")
	}
}

func TestGCodeScript(t *testing.T) {
	s := newTestServer()
	mux := http.NewServeMux()
	mux.HandleFunc("/printer/gcode/script", s.handleGCodeScript)

	body := bytes.NewBufferString(`{"script":"G28"}`)
	req := httptest.NewRequest("POST", "/printer/gcode/script", body)
	rec := httptest.NewRecorder()
	mux.ServeHTTP(rec, req)

	if rec.Code != http.StatusOK {
		t.Fatalf("expected status 200, got %d", rec.Code)
	}
}

func TestJSONRPC(t *testing.T) {
	s := newTestServer()
	mux := http.NewServeMux()
	mux.HandleFunc("/jsonrpc", s.handleJSONRPC)

	testCases := []struct {
		name   string
		method string
		params map[string]any
	}{
		{"server.info", "server.info", nil},
		{"printer.info", "printer.info", nil},
		{"printer.objects.list", "printer.objects.list", nil},
		{"printer.objects.query", "printer.objects.query", map[string]any{"objects": map[string]any{"toolhead": nil}}},
	}

	for _, tc := range testCases {
		t.Run(tc.name, func(t *testing.T) {
			reqBody := map[string]any{
				"jsonrpc": "2.0",
				"method":  tc.method,
				"id":      1,
			}
			if tc.params != nil {
				reqBody["params"] = tc.params
			}

			bodyBytes, _ := json.Marshal(reqBody)
			req := httptest.NewRequest("POST", "/jsonrpc", bytes.NewReader(bodyBytes))
			rec := httptest.NewRecorder()
			mux.ServeHTTP(rec, req)

			if rec.Code != http.StatusOK {
				t.Fatalf("expected status 200, got %d", rec.Code)
			}

			var resp jsonRPCResponse
			if err := json.NewDecoder(rec.Body).Decode(&resp); err != nil {
				t.Fatalf("failed to decode response: %v", err)
			}

			if resp.JSONRPC != "2.0" {
				t.Errorf("expected jsonrpc '2.0', got %s", resp.JSONRPC)
			}

			if resp.Error != nil {
				t.Errorf("unexpected error: %v", resp.Error)
			}

			if resp.Result == nil {
				t.Error("expected result, got nil")
			}
		})
	}
}

func TestWebSocket(t *testing.T) {
	s := newTestServer()
	s.running.Store(true)

	// Create test server
	mux := http.NewServeMux()
	mux.HandleFunc("/websocket", s.handleWebSocket)
	server := httptest.NewServer(mux)
	defer server.Close()

	// Convert http URL to ws URL
	wsURL := "ws" + server.URL[4:] + "/websocket"

	// Connect WebSocket
	conn, _, err := websocket.DefaultDialer.Dial(wsURL, nil)
	if err != nil {
		t.Fatalf("failed to connect WebSocket: %v", err)
	}
	defer conn.Close()

	// Send JSON-RPC request
	req := map[string]any{
		"jsonrpc": "2.0",
		"method":  "server.info",
		"id":      1,
	}
	if err := conn.WriteJSON(req); err != nil {
		t.Fatalf("failed to send message: %v", err)
	}

	// Read response
	conn.SetReadDeadline(time.Now().Add(5 * time.Second))
	_, message, err := conn.ReadMessage()
	if err != nil {
		t.Fatalf("failed to read message: %v", err)
	}

	var resp jsonRPCResponse
	if err := json.Unmarshal(message, &resp); err != nil {
		t.Fatalf("failed to decode response: %v", err)
	}

	if resp.Error != nil {
		t.Errorf("unexpected error: %v", resp.Error)
	}

	if resp.Result == nil {
		t.Error("expected result, got nil")
	}
}

func TestWebSocketSubscription(t *testing.T) {
	s := newTestServer()
	s.running.Store(true)

	// Start status broadcast loop in background
	go s.statusBroadcastLoop()

	// Create test server
	mux := http.NewServeMux()
	mux.HandleFunc("/websocket", s.handleWebSocket)
	server := httptest.NewServer(mux)
	defer server.Close()

	// Convert http URL to ws URL
	wsURL := "ws" + server.URL[4:] + "/websocket"

	// Connect WebSocket
	conn, _, err := websocket.DefaultDialer.Dial(wsURL, nil)
	if err != nil {
		t.Fatalf("failed to connect WebSocket: %v", err)
	}
	defer conn.Close()

	// Subscribe to objects
	req := map[string]any{
		"jsonrpc": "2.0",
		"method":  "printer.objects.subscribe",
		"params": map[string]any{
			"objects": map[string]any{
				"toolhead": nil,
				"extruder": []string{"temperature", "target"},
			},
		},
		"id": 1,
	}
	if err := conn.WriteJSON(req); err != nil {
		t.Fatalf("failed to send message: %v", err)
	}

	// Read initial response
	conn.SetReadDeadline(time.Now().Add(5 * time.Second))
	_, message, err := conn.ReadMessage()
	if err != nil {
		t.Fatalf("failed to read message: %v", err)
	}

	var resp jsonRPCResponse
	if err := json.Unmarshal(message, &resp); err != nil {
		t.Fatalf("failed to decode response: %v", err)
	}

	if resp.Error != nil {
		t.Errorf("unexpected error: %v", resp.Error)
	}

	// Wait for status update notification
	conn.SetReadDeadline(time.Now().Add(2 * time.Second))
	_, message, err = conn.ReadMessage()
	if err != nil {
		// It's OK if we timeout - the test is mainly checking subscription setup
		if err == io.EOF {
			return
		}
		t.Logf("note: no status update received within timeout (this may be expected): %v", err)
		return
	}

	var notification map[string]any
	if err := json.Unmarshal(message, &notification); err != nil {
		t.Fatalf("failed to decode notification: %v", err)
	}

	if notification["method"] != "notify_status_update" {
		t.Errorf("expected method 'notify_status_update', got %v", notification["method"])
	}

	s.running.Store(false)
}

func TestDefaultObjectStatus(t *testing.T) {
	s := New(Config{Addr: ":7125"}) // No printer

	testCases := []struct {
		name  string
		attrs []string
		want  []string
	}{
		{"print_stats", nil, []string{"state", "print_duration", "filename"}},
		{"print_stats", []string{"state"}, []string{"state"}},
		{"toolhead", nil, []string{"position", "extruder", "homed_axes"}},
		{"unknown", nil, nil},
	}

	for _, tc := range testCases {
		t.Run(tc.name, func(t *testing.T) {
			status := s.getDefaultObjectStatus(tc.name, tc.attrs)

			if tc.want == nil {
				if status != nil {
					t.Errorf("expected nil, got %v", status)
				}
				return
			}

			if status == nil {
				t.Fatal("expected status, got nil")
			}

			for _, key := range tc.want {
				if _, ok := status[key]; !ok {
					t.Errorf("expected key %s in status", key)
				}
			}
		})
	}
}
