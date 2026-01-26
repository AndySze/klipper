// Package moonraker provides a Moonraker-compatible API server.
// This allows Fluidd/Mainsail frontends to connect to the Go Klipper host.
package moonraker

import (
	"encoding/json"
	"fmt"
	"log"
	"net/http"
	"os"
	"sync"
	"sync/atomic"
	"time"

	"github.com/gorilla/websocket"
)

// Server provides a Moonraker-compatible API server.
type Server struct {
	// Printer interface for status queries
	printer PrinterInterface

	// HTTP server
	httpServer *http.Server
	addr       string

	// WebSocket management
	wsUpgrader websocket.Upgrader
	wsClients  map[int64]*WSClient
	wsClientMu sync.RWMutex
	nextWSID   int64

	// Status subscriptions
	subscriptions map[int64]map[string][]string // clientID -> object -> attributes
	subMu         sync.RWMutex

	// File and history managers
	fileManager    *FileManager
	historyManager *HistoryManager

	// Database storage (namespace -> key -> value)
	database   map[string]map[string]any
	databaseMu sync.RWMutex

	// Server state
	running   atomic.Bool
	startTime time.Time
}

// PrinterInterface defines the interface for printer status queries.
type PrinterInterface interface {
	// GetObjectsList returns list of available printer objects.
	GetObjectsList() []string

	// GetObjectStatus returns the status of a printer object.
	// If attrs is nil, return all attributes.
	GetObjectStatus(name string, attrs []string) map[string]any

	// ExecuteGCode executes a G-code script.
	ExecuteGCode(script string) error

	// EmergencyStop triggers an emergency stop.
	EmergencyStop()

	// GetKlippyState returns the current Klippy state.
	// One of: "disconnected", "startup", "ready", "error", "shutdown"
	GetKlippyState() string
}

// Config holds server configuration.
type Config struct {
	// HTTP address to listen on (e.g., ":7125")
	Addr string

	// Printer interface for status queries
	Printer PrinterInterface
}

// New creates a new Moonraker-compatible server.
func New(cfg Config) *Server {
	s := &Server{
		printer:        cfg.Printer,
		addr:           cfg.Addr,
		wsClients:      make(map[int64]*WSClient),
		subscriptions:  make(map[int64]map[string][]string),
		fileManager:    NewFileManager(),
		historyManager: NewHistoryManager(),
		database:       make(map[string]map[string]any),
		startTime:      time.Now(),
	}

	s.wsUpgrader = websocket.Upgrader{
		CheckOrigin: func(r *http.Request) bool {
			return true // Allow all origins for development
		},
	}

	return s
}

// FileManager returns the file manager.
func (s *Server) FileManager() *FileManager {
	return s.fileManager
}

// HistoryManager returns the history manager.
func (s *Server) HistoryManager() *HistoryManager {
	return s.historyManager
}

// Start starts the API server.
func (s *Server) Start() error {
	mux := http.NewServeMux()

	// JSON-RPC endpoint
	mux.HandleFunc("/jsonrpc", s.handleJSONRPC)

	// WebSocket endpoint
	mux.HandleFunc("/websocket", s.handleWebSocket)

	// REST-style endpoints (alternative to JSON-RPC)
	mux.HandleFunc("/server/info", s.handleServerInfo)
	mux.HandleFunc("/printer/info", s.handlePrinterInfo)
	mux.HandleFunc("/printer/objects/list", s.handleObjectsList)
	mux.HandleFunc("/printer/objects/query", s.handleObjectsQuery)
	mux.HandleFunc("/printer/gcode/script", s.handleGCodeScript)
	mux.HandleFunc("/printer/emergency_stop", s.handleEmergencyStop)

	// Access/authentication endpoints
	mux.HandleFunc("/access/oneshot_token", s.handleOneshotToken)

	// Database endpoints (for Fluidd/Mainsail settings storage)
	mux.HandleFunc("/server/database/item", s.handleDatabaseItem)
	mux.HandleFunc("/server/database/list", s.handleDatabaseList)

	// File management endpoints
	s.fileManager.RegisterFileEndpoints(mux)

	// History endpoints
	s.historyManager.RegisterHistoryEndpoints(mux)

	// Wrap with CORS middleware
	corsHandler := s.corsMiddleware(mux)

	s.httpServer = &http.Server{
		Addr:    s.addr,
		Handler: corsHandler,
	}

	s.running.Store(true)
	log.Printf("Moonraker API server starting on %s", s.addr)

	go s.statusBroadcastLoop()

	return s.httpServer.ListenAndServe()
}

// Stop stops the API server.
func (s *Server) Stop() error {
	s.running.Store(false)

	// Close all WebSocket clients
	s.wsClientMu.Lock()
	for _, client := range s.wsClients {
		client.Close()
	}
	s.wsClients = make(map[int64]*WSClient)
	s.wsClientMu.Unlock()

	if s.httpServer != nil {
		return s.httpServer.Close()
	}
	return nil
}

// JSON-RPC 2.0 structures

type jsonRPCRequest struct {
	JSONRPC string         `json:"jsonrpc"`
	Method  string         `json:"method"`
	Params  map[string]any `json:"params,omitempty"`
	ID      any            `json:"id,omitempty"`
}

type jsonRPCResponse struct {
	JSONRPC string         `json:"jsonrpc"`
	Result  any            `json:"result,omitempty"`
	Error   *jsonRPCError  `json:"error,omitempty"`
	ID      any            `json:"id,omitempty"`
}

type jsonRPCError struct {
	Code    int    `json:"code"`
	Message string `json:"message"`
}

// handleJSONRPC handles JSON-RPC 2.0 requests.
func (s *Server) handleJSONRPC(w http.ResponseWriter, r *http.Request) {
	if r.Method != http.MethodPost {
		http.Error(w, "Method not allowed", http.StatusMethodNotAllowed)
		return
	}

	var req jsonRPCRequest
	if err := json.NewDecoder(r.Body).Decode(&req); err != nil {
		s.writeJSONRPCError(w, nil, -32700, "Parse error")
		return
	}

	result, err := s.dispatchMethod(req.Method, req.Params, nil)
	if err != nil {
		s.writeJSONRPCError(w, req.ID, -32000, err.Error())
		return
	}

	s.writeJSONRPCResult(w, req.ID, result)
}

// dispatchMethod routes a method call to the appropriate handler.
func (s *Server) dispatchMethod(method string, params map[string]any, client *WSClient) (any, error) {
	switch method {
	case "server.info":
		return s.methodServerInfo()
	case "printer.info":
		return s.methodPrinterInfo()
	case "printer.objects.list":
		return s.methodObjectsList()
	case "printer.objects.query":
		return s.methodObjectsQuery(params)
	case "printer.objects.subscribe":
		return s.methodObjectsSubscribe(params, client)
	case "printer.gcode.script":
		return s.methodGCodeScript(params)
	case "printer.emergency_stop":
		return s.methodEmergencyStop()
	case "server.connection.identify":
		return s.methodIdentify(params)
	default:
		return nil, fmt.Errorf("method not found: %s", method)
	}
}

// Method implementations

func (s *Server) methodServerInfo() (any, error) {
	hostname, _ := os.Hostname()
	klippyState := "ready"
	if s.printer != nil {
		klippyState = s.printer.GetKlippyState()
	}

	return map[string]any{
		"klippy_connected": klippyState == "ready",
		"klippy_state":     klippyState,
		"components": []string{
			"klippy_apis",
			"history",
			"octoprint_compat",
		},
		"failed_components": []string{},
		"registered_directories": []string{
			"gcodes",
			"config",
		},
		"warnings":          []string{},
		"websocket_count":   len(s.wsClients),
		"moonraker_version": "v0.8.0-klipper-go",
		"api_version":       []int{1, 5, 0},
		"api_version_string": "1.5.0",
		"hostname":          hostname,
	}, nil
}

func (s *Server) methodPrinterInfo() (any, error) {
	hostname, _ := os.Hostname()
	state := "ready"
	stateMessage := "Printer is ready"
	if s.printer != nil {
		state = s.printer.GetKlippyState()
		if state != "ready" {
			stateMessage = "Printer is not ready"
		}
	}

	return map[string]any{
		"state":            state,
		"state_message":    stateMessage,
		"hostname":         hostname,
		"software_version": "klipper-go-0.1.0",
		"cpu_info":         "",
		"klipper_path":     "/home/pi/klipper",
		"python_path":      "/home/pi/klippy-env/bin/python",
		"log_file":         "/home/pi/printer_data/logs/klippy.log",
		"config_file":      "/home/pi/printer_data/config/printer.cfg",
	}, nil
}

func (s *Server) methodObjectsList() (any, error) {
	objects := []string{
		"webhooks",
		"configfile",
		"mcu",
		"gcode_move",
		"print_stats",
		"virtual_sdcard",
		"pause_resume",
		"display_status",
		"heaters",
		"heater_bed",
		"fan",
		"toolhead",
		"extruder",
		"motion_report",
		"query_endstops",
		"system_stats",
		"idle_timeout",
	}

	if s.printer != nil {
		objects = s.printer.GetObjectsList()
	}

	return map[string]any{"objects": objects}, nil
}

func (s *Server) methodObjectsQuery(params map[string]any) (any, error) {
	objectsParam, ok := params["objects"]
	if !ok {
		return nil, fmt.Errorf("missing 'objects' parameter")
	}

	objects, ok := objectsParam.(map[string]any)
	if !ok {
		return nil, fmt.Errorf("'objects' must be an object")
	}

	result := make(map[string]any)
	eventtime := float64(time.Since(s.startTime).Milliseconds()) / 1000.0

	for objName, attrsVal := range objects {
		var attrs []string

		// Parse attributes: null means all, array means specific
		if attrList, ok := attrsVal.([]any); ok {
			for _, attr := range attrList {
				if attrStr, ok := attr.(string); ok {
					attrs = append(attrs, attrStr)
				}
			}
		}
		// nil/null means all attributes

		var status map[string]any
		if s.printer != nil {
			status = s.printer.GetObjectStatus(objName, attrs)
		} else {
			status = s.getDefaultObjectStatus(objName, attrs)
		}

		if status != nil {
			result[objName] = status
		}
	}

	return map[string]any{
		"eventtime": eventtime,
		"status":    result,
	}, nil
}

func (s *Server) methodObjectsSubscribe(params map[string]any, client *WSClient) (any, error) {
	if client == nil {
		return nil, fmt.Errorf("subscription requires WebSocket connection")
	}

	objectsParam, ok := params["objects"]
	if !ok {
		return nil, fmt.Errorf("missing 'objects' parameter")
	}

	objects, ok := objectsParam.(map[string]any)
	if !ok {
		return nil, fmt.Errorf("'objects' must be an object")
	}

	// Store subscription
	s.subMu.Lock()
	s.subscriptions[client.id] = make(map[string][]string)
	for objName, attrsVal := range objects {
		var attrs []string
		if attrList, ok := attrsVal.([]any); ok {
			for _, attr := range attrList {
				if attrStr, ok := attr.(string); ok {
					attrs = append(attrs, attrStr)
				}
			}
		}
		s.subscriptions[client.id][objName] = attrs
	}
	s.subMu.Unlock()

	// Return initial status
	return s.methodObjectsQuery(params)
}

func (s *Server) methodGCodeScript(params map[string]any) (any, error) {
	script, ok := params["script"].(string)
	if !ok {
		return nil, fmt.Errorf("missing 'script' parameter")
	}

	if s.printer != nil {
		if err := s.printer.ExecuteGCode(script); err != nil {
			return nil, err
		}
	} else {
		log.Printf("Moonraker: gcode script (no printer): %s", script)
	}

	return map[string]any{}, nil
}

func (s *Server) methodEmergencyStop() (any, error) {
	log.Println("Moonraker: emergency stop requested")
	if s.printer != nil {
		s.printer.EmergencyStop()
	}
	return map[string]any{}, nil
}

func (s *Server) methodIdentify(params map[string]any) (any, error) {
	clientName := "unknown"
	if name, ok := params["client_name"].(string); ok {
		clientName = name
	}
	log.Printf("Moonraker: client identified as %s", clientName)
	return map[string]any{
		"connection_id": atomic.AddInt64(&s.nextWSID, 0),
	}, nil
}

// getDefaultObjectStatus returns default status when no printer is connected.
func (s *Server) getDefaultObjectStatus(name string, attrs []string) map[string]any {
	defaults := map[string]map[string]any{
		"webhooks": {
			"state":         "ready",
			"state_message": "Printer is ready",
		},
		"print_stats": {
			"state":          "standby",
			"print_duration": 0.0,
			"filename":       "",
			"total_duration": 0.0,
			"filament_used":  0.0,
			"message":        "",
		},
		"virtual_sdcard": {
			"file_path":      "",
			"progress":       0.0,
			"is_active":      false,
			"file_position":  0,
			"file_size":      0,
		},
		"pause_resume": {
			"is_paused": false,
		},
		"display_status": {
			"progress": 0.0,
			"message":  "",
		},
		"gcode_move": {
			"speed_factor":           1.0,
			"speed":                  0.0,
			"extrude_factor":         1.0,
			"absolute_coordinates":   true,
			"absolute_extrude":       false,
			"homing_origin":          []float64{0, 0, 0, 0},
			"position":               []float64{0, 0, 0, 0},
			"gcode_position":         []float64{0, 0, 0, 0},
		},
		"toolhead": {
			"position":           []float64{0, 0, 0, 0},
			"extruder":           "extruder",
			"homed_axes":         "",
			"print_time":         0.0,
			"estimated_print_time": 0.0,
			"max_velocity":       500.0,
			"max_accel":          3000.0,
			"max_accel_to_decel": 1500.0,
			"square_corner_velocity": 5.0,
		},
		"extruder": {
			"temperature":    0.0,
			"target":         0.0,
			"power":          0.0,
			"can_extrude":    false,
			"pressure_advance": 0.0,
			"smooth_time":    0.04,
		},
		"heater_bed": {
			"temperature": 0.0,
			"target":      0.0,
			"power":       0.0,
		},
		"fan": {
			"speed": 0.0,
			"rpm":   nil,
		},
		"idle_timeout": {
			"state":         "Idle",
			"printing_time": 0.0,
		},
		"system_stats": {
			"sysload":  0.0,
			"cputime":  0.0,
			"memavail": 0,
		},
		"motion_report": {
			"live_position":      []float64{0, 0, 0, 0},
			"live_velocity":      0.0,
			"live_extruder_velocity": 0.0,
		},
	}

	status, ok := defaults[name]
	if !ok {
		return nil
	}

	// Filter attributes if specified
	if len(attrs) > 0 {
		filtered := make(map[string]any)
		for _, attr := range attrs {
			if val, exists := status[attr]; exists {
				filtered[attr] = val
			}
		}
		return filtered
	}

	return status
}

// REST endpoint handlers

func (s *Server) handleServerInfo(w http.ResponseWriter, r *http.Request) {
	result, err := s.methodServerInfo()
	if err != nil {
		s.writeJSONError(w, err)
		return
	}
	s.writeJSON(w, map[string]any{"result": result})
}

func (s *Server) handlePrinterInfo(w http.ResponseWriter, r *http.Request) {
	result, err := s.methodPrinterInfo()
	if err != nil {
		s.writeJSONError(w, err)
		return
	}
	s.writeJSON(w, map[string]any{"result": result})
}

func (s *Server) handleObjectsList(w http.ResponseWriter, r *http.Request) {
	result, err := s.methodObjectsList()
	if err != nil {
		s.writeJSONError(w, err)
		return
	}
	s.writeJSON(w, map[string]any{"result": result})
}

func (s *Server) handleObjectsQuery(w http.ResponseWriter, r *http.Request) {
	if r.Method != http.MethodPost {
		http.Error(w, "Method not allowed", http.StatusMethodNotAllowed)
		return
	}

	var params map[string]any
	if err := json.NewDecoder(r.Body).Decode(&params); err != nil {
		s.writeJSONError(w, err)
		return
	}

	result, err := s.methodObjectsQuery(params)
	if err != nil {
		s.writeJSONError(w, err)
		return
	}
	s.writeJSON(w, map[string]any{"result": result})
}

func (s *Server) handleGCodeScript(w http.ResponseWriter, r *http.Request) {
	if r.Method != http.MethodPost {
		http.Error(w, "Method not allowed", http.StatusMethodNotAllowed)
		return
	}

	var params map[string]any
	if err := json.NewDecoder(r.Body).Decode(&params); err != nil {
		s.writeJSONError(w, err)
		return
	}

	result, err := s.methodGCodeScript(params)
	if err != nil {
		s.writeJSONError(w, err)
		return
	}
	s.writeJSON(w, map[string]any{"result": result})
}

func (s *Server) handleEmergencyStop(w http.ResponseWriter, r *http.Request) {
	result, err := s.methodEmergencyStop()
	if err != nil {
		s.writeJSONError(w, err)
		return
	}
	s.writeJSON(w, map[string]any{"result": result})
}

func (s *Server) handleOneshotToken(w http.ResponseWriter, r *http.Request) {
	// Generate a simple oneshot token for authentication
	// In a full implementation, this would be a proper random token with expiry
	token := fmt.Sprintf("klipper-go-%d", time.Now().UnixNano())
	s.writeJSON(w, map[string]any{"result": token})
}

func (s *Server) handleDatabaseItem(w http.ResponseWriter, r *http.Request) {
	namespace := r.URL.Query().Get("namespace")
	key := r.URL.Query().Get("key")

	switch r.Method {
	case http.MethodGet:
		// Get item from database
		s.databaseMu.RLock()
		ns, nsExists := s.database[namespace]
		var value any
		if nsExists && key != "" {
			value = ns[key]
		} else if nsExists && key == "" {
			// Return entire namespace
			value = ns
		}
		s.databaseMu.RUnlock()

		if value == nil {
			// Return empty object for non-existent keys (Fluidd expects this)
			s.writeJSON(w, map[string]any{"result": map[string]any{"namespace": namespace, "key": key, "value": map[string]any{}}})
		} else {
			s.writeJSON(w, map[string]any{"result": map[string]any{"namespace": namespace, "key": key, "value": value}})
		}

	case http.MethodPost:
		// Set item in database
		var body map[string]any
		if err := json.NewDecoder(r.Body).Decode(&body); err != nil {
			s.writeJSONError(w, err)
			return
		}

		value := body["value"]

		s.databaseMu.Lock()
		if s.database[namespace] == nil {
			s.database[namespace] = make(map[string]any)
		}
		s.database[namespace][key] = value
		s.databaseMu.Unlock()

		s.writeJSON(w, map[string]any{"result": map[string]any{"namespace": namespace, "key": key, "value": value}})

	case http.MethodDelete:
		// Delete item from database
		s.databaseMu.Lock()
		if ns, exists := s.database[namespace]; exists {
			delete(ns, key)
		}
		s.databaseMu.Unlock()

		s.writeJSON(w, map[string]any{"result": map[string]any{"namespace": namespace, "key": key}})

	default:
		http.Error(w, "Method not allowed", http.StatusMethodNotAllowed)
	}
}

func (s *Server) handleDatabaseList(w http.ResponseWriter, r *http.Request) {
	s.databaseMu.RLock()
	namespaces := make([]string, 0, len(s.database))
	for ns := range s.database {
		namespaces = append(namespaces, ns)
	}
	s.databaseMu.RUnlock()

	s.writeJSON(w, map[string]any{"result": map[string]any{"namespaces": namespaces}})
}

// CORS middleware to allow cross-origin requests from Fluidd/Mainsail
func (s *Server) corsMiddleware(next http.Handler) http.Handler {
	return http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
		// Set CORS headers
		w.Header().Set("Access-Control-Allow-Origin", "*")
		w.Header().Set("Access-Control-Allow-Methods", "GET, POST, PUT, DELETE, OPTIONS")
		w.Header().Set("Access-Control-Allow-Headers", "Content-Type, Authorization, X-Requested-With")
		w.Header().Set("Access-Control-Allow-Credentials", "true")

		// Handle preflight requests
		if r.Method == http.MethodOptions {
			w.WriteHeader(http.StatusOK)
			return
		}

		next.ServeHTTP(w, r)
	})
}

// JSON response helpers

func (s *Server) writeJSON(w http.ResponseWriter, data any) {
	w.Header().Set("Content-Type", "application/json")
	json.NewEncoder(w).Encode(data)
}

func (s *Server) writeJSONError(w http.ResponseWriter, err error) {
	w.Header().Set("Content-Type", "application/json")
	w.WriteHeader(http.StatusBadRequest)
	json.NewEncoder(w).Encode(map[string]any{
		"error": map[string]any{
			"code":    -32000,
			"message": err.Error(),
		},
	})
}

func (s *Server) writeJSONRPCResult(w http.ResponseWriter, id any, result any) {
	w.Header().Set("Content-Type", "application/json")
	json.NewEncoder(w).Encode(jsonRPCResponse{
		JSONRPC: "2.0",
		Result:  result,
		ID:      id,
	})
}

func (s *Server) writeJSONRPCError(w http.ResponseWriter, id any, code int, message string) {
	w.Header().Set("Content-Type", "application/json")
	json.NewEncoder(w).Encode(jsonRPCResponse{
		JSONRPC: "2.0",
		Error:   &jsonRPCError{Code: code, Message: message},
		ID:      id,
	})
}

// WSClient represents a WebSocket client connection.
type WSClient struct {
	id     int64
	conn   *websocket.Conn
	server *Server
	sendCh chan any
	done   chan struct{}
	mu     sync.Mutex
}

// newWSClient creates a new WebSocket client.
func (s *Server) newWSClient(conn *websocket.Conn) *WSClient {
	id := atomic.AddInt64(&s.nextWSID, 1)
	client := &WSClient{
		id:     id,
		conn:   conn,
		server: s,
		sendCh: make(chan any, 64),
		done:   make(chan struct{}),
	}
	return client
}

// Send sends a message to the client.
func (c *WSClient) Send(msg any) {
	select {
	case c.sendCh <- msg:
	case <-c.done:
	default:
		// Channel full, drop message
		log.Printf("Moonraker: dropping message to client %d (channel full)", c.id)
	}
}

// Close closes the client connection.
func (c *WSClient) Close() {
	c.mu.Lock()
	defer c.mu.Unlock()

	select {
	case <-c.done:
		return // Already closed
	default:
		close(c.done)
	}

	c.conn.Close()
}

// readPump reads messages from the WebSocket connection.
func (c *WSClient) readPump() {
	defer func() {
		c.server.removeClient(c)
		c.Close()
	}()

	c.conn.SetReadLimit(512 * 1024) // 512KB max message size
	c.conn.SetReadDeadline(time.Now().Add(60 * time.Second))
	c.conn.SetPongHandler(func(string) error {
		c.conn.SetReadDeadline(time.Now().Add(60 * time.Second))
		return nil
	})

	for {
		_, message, err := c.conn.ReadMessage()
		if err != nil {
			if websocket.IsUnexpectedCloseError(err, websocket.CloseGoingAway, websocket.CloseAbnormalClosure) {
				log.Printf("Moonraker: WebSocket read error: %v", err)
			}
			break
		}

		c.handleMessage(message)
	}
}

// writePump sends messages to the WebSocket connection.
func (c *WSClient) writePump() {
	ticker := time.NewTicker(30 * time.Second)
	defer func() {
		ticker.Stop()
		c.Close()
	}()

	for {
		select {
		case msg, ok := <-c.sendCh:
			c.conn.SetWriteDeadline(time.Now().Add(10 * time.Second))
			if !ok {
				c.conn.WriteMessage(websocket.CloseMessage, []byte{})
				return
			}

			if err := c.conn.WriteJSON(msg); err != nil {
				log.Printf("Moonraker: WebSocket write error: %v", err)
				return
			}

		case <-ticker.C:
			c.conn.SetWriteDeadline(time.Now().Add(10 * time.Second))
			if err := c.conn.WriteMessage(websocket.PingMessage, nil); err != nil {
				return
			}

		case <-c.done:
			return
		}
	}
}

// handleMessage processes an incoming WebSocket message.
func (c *WSClient) handleMessage(data []byte) {
	var req jsonRPCRequest
	if err := json.Unmarshal(data, &req); err != nil {
		c.sendError(nil, -32700, "Parse error")
		return
	}

	result, err := c.server.dispatchMethod(req.Method, req.Params, c)
	if err != nil {
		c.sendError(req.ID, -32000, err.Error())
		return
	}

	c.Send(jsonRPCResponse{
		JSONRPC: "2.0",
		Result:  result,
		ID:      req.ID,
	})
}

// sendError sends a JSON-RPC error response.
func (c *WSClient) sendError(id any, code int, message string) {
	c.Send(jsonRPCResponse{
		JSONRPC: "2.0",
		Error:   &jsonRPCError{Code: code, Message: message},
		ID:      id,
	})
}

// handleWebSocket handles WebSocket upgrade and connection.
func (s *Server) handleWebSocket(w http.ResponseWriter, r *http.Request) {
	conn, err := s.wsUpgrader.Upgrade(w, r, nil)
	if err != nil {
		log.Printf("Moonraker: WebSocket upgrade error: %v", err)
		return
	}

	client := s.newWSClient(conn)

	s.wsClientMu.Lock()
	s.wsClients[client.id] = client
	s.wsClientMu.Unlock()

	log.Printf("Moonraker: WebSocket client %d connected", client.id)

	// Start read and write pumps
	go client.writePump()

	// Send initial notifications after connection
	go func() {
		// Small delay to ensure client is ready
		time.Sleep(100 * time.Millisecond)

		// Send klippy_connected notification
		client.Send(map[string]any{
			"jsonrpc": "2.0",
			"method":  "notify_klippy_connected",
		})

		// Send klippy_ready notification if printer is ready
		klippyState := "ready"
		if s.printer != nil {
			klippyState = s.printer.GetKlippyState()
		}
		if klippyState == "ready" {
			client.Send(map[string]any{
				"jsonrpc": "2.0",
				"method":  "notify_klippy_ready",
			})
		}
	}()

	client.readPump() // Blocks until connection closes
}

// removeClient removes a client and cleans up its subscriptions.
func (s *Server) removeClient(client *WSClient) {
	s.wsClientMu.Lock()
	delete(s.wsClients, client.id)
	s.wsClientMu.Unlock()

	s.subMu.Lock()
	delete(s.subscriptions, client.id)
	s.subMu.Unlock()

	log.Printf("Moonraker: WebSocket client %d disconnected", client.id)
}

// statusBroadcastLoop periodically broadcasts status updates to subscribed clients.
func (s *Server) statusBroadcastLoop() {
	ticker := time.NewTicker(250 * time.Millisecond) // 4 Hz update rate
	defer ticker.Stop()

	for s.running.Load() {
		<-ticker.C
		s.broadcastStatusUpdates()
	}
}

// broadcastStatusUpdates sends status updates to all subscribed clients.
func (s *Server) broadcastStatusUpdates() {
	s.subMu.RLock()
	defer s.subMu.RUnlock()

	eventtime := float64(time.Since(s.startTime).Milliseconds()) / 1000.0

	for clientID, objects := range s.subscriptions {
		s.wsClientMu.RLock()
		client, ok := s.wsClients[clientID]
		s.wsClientMu.RUnlock()

		if !ok {
			continue
		}

		// Build status update for this client's subscriptions
		status := make(map[string]any)
		for objName, attrs := range objects {
			var objStatus map[string]any
			if s.printer != nil {
				objStatus = s.printer.GetObjectStatus(objName, attrs)
			} else {
				objStatus = s.getDefaultObjectStatus(objName, attrs)
			}
			if objStatus != nil {
				status[objName] = objStatus
			}
		}

		if len(status) == 0 {
			continue
		}

		// Send notification
		notification := map[string]any{
			"jsonrpc": "2.0",
			"method":  "notify_status_update",
			"params":  []any{status, eventtime},
		}

		client.Send(notification)
	}
}
