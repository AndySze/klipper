// Webhooks API server - port of klippy/webhooks.py
//
// Copyright (C) 2020 Eric Callahan <arksine.code@gmail.com>
// Copyright (C) 2025 Go port
//
// This file may be distributed under the terms of the GNU GPLv3 license.

package hosth4

import (
	"encoding/json"
	"fmt"
	"log"
	"net"
	"os"
	"sync"
)

// WebRequestError represents an error from a web request.
type WebRequestError struct {
	Message string `json:"message"`
}

func (e *WebRequestError) Error() string {
	return e.Message
}

// ToDict converts the error to a dictionary for JSON serialization.
func (e *WebRequestError) ToDict() map[string]any {
	return map[string]any{
		"error":   "WebRequestError",
		"message": e.Message,
	}
}

// WebRequest represents an incoming API request.
type WebRequest struct {
	client   *ClientConnection
	ID       any               `json:"id"`
	Method   string            `json:"method"`
	Params   map[string]any    `json:"params"`
	response any
	isError  bool
}

// Get retrieves a parameter from the request.
func (wr *WebRequest) Get(key string, defaultVal any) any {
	if val, ok := wr.Params[key]; ok {
		return val
	}
	return defaultVal
}

// GetStr retrieves a string parameter from the request.
func (wr *WebRequest) GetStr(key string, defaultVal string) string {
	if val, ok := wr.Params[key]; ok {
		if s, ok := val.(string); ok {
			return s
		}
	}
	return defaultVal
}

// GetInt retrieves an integer parameter from the request.
func (wr *WebRequest) GetInt(key string, defaultVal int) int {
	if val, ok := wr.Params[key]; ok {
		switch v := val.(type) {
		case int:
			return v
		case float64:
			return int(v)
		case int64:
			return int(v)
		}
	}
	return defaultVal
}

// GetFloat retrieves a float parameter from the request.
func (wr *WebRequest) GetFloat(key string, defaultVal float64) float64 {
	if val, ok := wr.Params[key]; ok {
		switch v := val.(type) {
		case float64:
			return v
		case int:
			return float64(v)
		case int64:
			return float64(v)
		}
	}
	return defaultVal
}

// GetDict retrieves a dict parameter from the request.
func (wr *WebRequest) GetDict(key string) map[string]any {
	if val, ok := wr.Params[key]; ok {
		if d, ok := val.(map[string]any); ok {
			return d
		}
	}
	return nil
}

// GetMethod returns the request method.
func (wr *WebRequest) GetMethod() string {
	return wr.Method
}

// GetClientConnection returns the client connection.
func (wr *WebRequest) GetClientConnection() *ClientConnection {
	return wr.client
}

// SetError sets an error response.
func (wr *WebRequest) SetError(err error) {
	wr.isError = true
	wr.response = map[string]any{
		"error":   "WebRequestError",
		"message": err.Error(),
	}
}

// Send sends a response.
func (wr *WebRequest) Send(data any) error {
	if wr.response != nil {
		return fmt.Errorf("multiple calls to send not allowed")
	}
	wr.response = data
	return nil
}

// Finish completes the request and returns the response.
func (wr *WebRequest) Finish() map[string]any {
	if wr.ID == nil {
		return nil
	}
	rtype := "result"
	if wr.isError {
		rtype = "error"
	}
	if wr.response == nil {
		wr.response = map[string]any{}
	}
	return map[string]any{
		"id":  wr.ID,
		rtype: wr.response,
	}
}

// EndpointCallback is a function that handles an API endpoint.
type EndpointCallback func(wr *WebRequest) error

// WebHooks manages the webhooks API server.
type WebHooks struct {
	rt        *runtime
	endpoints map[string]EndpointCallback
	mu        sync.RWMutex

	// Server state
	listener net.Listener
	clients  map[int]*ClientConnection
	clientMu sync.Mutex
	nextID   int
	running  bool
}

// newWebHooks creates a new webhooks manager.
func newWebHooks(rt *runtime) *WebHooks {
	wh := &WebHooks{
		rt:        rt,
		endpoints: make(map[string]EndpointCallback),
		clients:   make(map[int]*ClientConnection),
	}

	// Register built-in endpoints
	wh.RegisterEndpoint("list_endpoints", wh.handleListEndpoints)
	wh.RegisterEndpoint("info", wh.handleInfoRequest)
	wh.RegisterEndpoint("emergency_stop", wh.handleEmergencyStop)
	wh.RegisterEndpoint("objects/list", wh.handleObjectsList)
	wh.RegisterEndpoint("objects/query", wh.handleObjectsQuery)
	wh.RegisterEndpoint("gcode/script", wh.handleGCodeScript)

	return wh
}

// RegisterEndpoint registers an API endpoint.
func (wh *WebHooks) RegisterEndpoint(path string, callback EndpointCallback) error {
	wh.mu.Lock()
	defer wh.mu.Unlock()

	if _, exists := wh.endpoints[path]; exists {
		return fmt.Errorf("path already registered: %s", path)
	}
	wh.endpoints[path] = callback
	return nil
}

// GetCallback returns the callback for a path.
func (wh *WebHooks) GetCallback(path string) (EndpointCallback, error) {
	wh.mu.RLock()
	defer wh.mu.RUnlock()

	cb, exists := wh.endpoints[path]
	if !exists {
		return nil, fmt.Errorf("no registered callback for path '%s'", path)
	}
	return cb, nil
}

// StartServer starts the API server on a Unix socket.
func (wh *WebHooks) StartServer(socketPath string) error {
	// Remove existing socket
	os.Remove(socketPath)

	listener, err := net.Listen("unix", socketPath)
	if err != nil {
		return fmt.Errorf("failed to listen on %s: %w", socketPath, err)
	}
	wh.listener = listener
	wh.running = true

	go wh.acceptLoop()
	log.Printf("Webhooks server listening on %s", socketPath)
	return nil
}

// StopServer stops the API server.
func (wh *WebHooks) StopServer() {
	wh.running = false
	if wh.listener != nil {
		wh.listener.Close()
	}

	wh.clientMu.Lock()
	for _, client := range wh.clients {
		client.Close()
	}
	wh.clients = make(map[int]*ClientConnection)
	wh.clientMu.Unlock()
}

// acceptLoop accepts new connections.
func (wh *WebHooks) acceptLoop() {
	for wh.running {
		conn, err := wh.listener.Accept()
		if err != nil {
			if wh.running {
				log.Printf("Accept error: %v", err)
			}
			continue
		}

		client := newClientConnection(wh, conn)
		wh.clientMu.Lock()
		wh.nextID++
		client.id = wh.nextID
		wh.clients[client.id] = client
		wh.clientMu.Unlock()

		go client.readLoop()
	}
}

// removeClient removes a client from the manager.
func (wh *WebHooks) removeClient(id int) {
	wh.clientMu.Lock()
	delete(wh.clients, id)
	wh.clientMu.Unlock()
}

// GetStatus returns the webhooks status.
func (wh *WebHooks) GetStatus() map[string]any {
	state := "ready"
	stateMessage := "Printer is ready"

	return map[string]any{
		"state":         state,
		"state_message": stateMessage,
	}
}

// Built-in endpoint handlers

func (wh *WebHooks) handleListEndpoints(wr *WebRequest) error {
	wh.mu.RLock()
	endpoints := make([]string, 0, len(wh.endpoints))
	for path := range wh.endpoints {
		endpoints = append(endpoints, path)
	}
	wh.mu.RUnlock()

	return wr.Send(map[string]any{"endpoints": endpoints})
}

func (wh *WebHooks) handleInfoRequest(wr *WebRequest) error {
	hostname, _ := os.Hostname()
	response := map[string]any{
		"state":            "ready",
		"state_message":    "Printer is ready",
		"hostname":         hostname,
		"software_version": "klipper-go-0.1",
		"cpu_info":         "",
	}
	return wr.Send(response)
}

func (wh *WebHooks) handleEmergencyStop(wr *WebRequest) error {
	log.Println("Emergency stop requested via webhooks")
	// In a full implementation, this would trigger a printer shutdown
	return wr.Send(map[string]any{})
}

func (wh *WebHooks) handleObjectsList(wr *WebRequest) error {
	// Return list of objects that have get_status methods
	objects := []string{
		"print_stats",
		"virtual_sdcard",
		"pause_resume",
		"gcode_move",
		"toolhead",
		"webhooks",
	}
	return wr.Send(map[string]any{"objects": objects})
}

func (wh *WebHooks) handleObjectsQuery(wr *WebRequest) error {
	objects := wr.GetDict("objects")
	if objects == nil {
		return fmt.Errorf("missing 'objects' parameter")
	}

	result := make(map[string]any)

	for objName, attrs := range objects {
		status := wh.getObjectStatus(objName)
		if status == nil {
			continue
		}

		// Filter to requested attributes if specified
		if attrList, ok := attrs.([]any); ok && len(attrList) > 0 {
			filtered := make(map[string]any)
			for _, attr := range attrList {
				if attrStr, ok := attr.(string); ok {
					if val, exists := status[attrStr]; exists {
						filtered[attrStr] = val
					}
				}
			}
			result[objName] = filtered
		} else {
			result[objName] = status
		}
	}

	return wr.Send(map[string]any{
		"eventtime": 0.0,
		"status":    result,
	})
}

func (wh *WebHooks) handleGCodeScript(wr *WebRequest) error {
	script := wr.GetStr("script", "")
	if script == "" {
		return fmt.Errorf("missing 'script' parameter")
	}

	// In a full implementation, this would run the gcode
	log.Printf("Webhooks gcode script: %s", script)
	return wr.Send(map[string]any{})
}

// getObjectStatus returns the status of a named object.
func (wh *WebHooks) getObjectStatus(name string) map[string]any {
	switch name {
	case "print_stats":
		if wh.rt.printStats != nil {
			return wh.rt.printStats.GetStatus()
		}
	case "virtual_sdcard":
		if wh.rt.sdcard != nil {
			return wh.rt.sdcard.GetStatus()
		}
	case "pause_resume":
		if wh.rt.pauseResume != nil {
			return wh.rt.pauseResume.GetStatus()
		}
	case "gcode_move":
		if wh.rt.gm != nil {
			return wh.rt.gm.GetStatus()
		}
	case "webhooks":
		return wh.GetStatus()
	}
	return nil
}

// ClientConnection represents a connected API client.
type ClientConnection struct {
	wh     *WebHooks
	conn   net.Conn
	id     int
	closed bool
	mu     sync.Mutex
}

// newClientConnection creates a new client connection.
func newClientConnection(wh *WebHooks, conn net.Conn) *ClientConnection {
	return &ClientConnection{
		wh:   wh,
		conn: conn,
	}
}

// Close closes the client connection.
func (cc *ClientConnection) Close() {
	cc.mu.Lock()
	if cc.closed {
		cc.mu.Unlock()
		return
	}
	cc.closed = true
	cc.mu.Unlock()

	cc.conn.Close()
	cc.wh.removeClient(cc.id)
}

// IsClosed returns true if the connection is closed.
func (cc *ClientConnection) IsClosed() bool {
	cc.mu.Lock()
	defer cc.mu.Unlock()
	return cc.closed
}

// Send sends data to the client.
func (cc *ClientConnection) Send(data any) error {
	cc.mu.Lock()
	if cc.closed {
		cc.mu.Unlock()
		return fmt.Errorf("connection closed")
	}
	cc.mu.Unlock()

	jsonData, err := json.Marshal(data)
	if err != nil {
		return fmt.Errorf("json encode error: %w", err)
	}

	// Append message separator (ETX byte)
	jsonData = append(jsonData, 0x03)

	_, err = cc.conn.Write(jsonData)
	return err
}

// readLoop reads and processes incoming messages.
func (cc *ClientConnection) readLoop() {
	defer cc.Close()

	buffer := make([]byte, 4096)
	var partial []byte

	for {
		n, err := cc.conn.Read(buffer)
		if err != nil {
			return
		}

		// Append to partial buffer
		partial = append(partial, buffer[:n]...)

		// Process complete messages (separated by ETX byte 0x03)
		for {
			idx := -1
			for i, b := range partial {
				if b == 0x03 {
					idx = i
					break
				}
			}
			if idx < 0 {
				break
			}

			msg := partial[:idx]
			partial = partial[idx+1:]

			// Process the message
			cc.processMessage(msg)
		}
	}
}

// processMessage processes a single incoming message.
func (cc *ClientConnection) processMessage(msg []byte) {
	var request struct {
		ID     any            `json:"id"`
		Method string         `json:"method"`
		Params map[string]any `json:"params"`
	}

	if err := json.Unmarshal(msg, &request); err != nil {
		log.Printf("Webhooks: error decoding request: %v", err)
		return
	}

	wr := &WebRequest{
		client: cc,
		ID:     request.ID,
		Method: request.Method,
		Params: request.Params,
	}
	if wr.Params == nil {
		wr.Params = make(map[string]any)
	}

	// Find and execute callback
	callback, err := cc.wh.GetCallback(wr.Method)
	if err != nil {
		wr.SetError(err)
	} else {
		if err := callback(wr); err != nil {
			wr.SetError(err)
		}
	}

	// Send response
	result := wr.Finish()
	if result != nil {
		cc.Send(result)
	}
}
