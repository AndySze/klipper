// Printer adapter for hosth4 runtime integration.
package moonraker

import (
	"strings"
	"sync"
)

// PrinterAdapter adapts hosth4 runtime to PrinterInterface.
type PrinterAdapter struct {
	mu sync.RWMutex

	// Status providers for different objects
	statusProviders map[string]StatusProvider

	// GCode executor
	gcodeExecutor func(script string) error

	// Emergency stop handler
	emergencyStopHandler func()

	// Klippy state getter
	klippyStateGetter func() string
}

// StatusProvider returns the status of a printer object.
type StatusProvider func(attrs []string) map[string]any

// NewPrinterAdapter creates a new printer adapter.
func NewPrinterAdapter() *PrinterAdapter {
	return &PrinterAdapter{
		statusProviders: make(map[string]StatusProvider),
	}
}

// RegisterStatusProvider registers a status provider for an object.
func (pa *PrinterAdapter) RegisterStatusProvider(name string, provider StatusProvider) {
	pa.mu.Lock()
	defer pa.mu.Unlock()
	pa.statusProviders[name] = provider
}

// UnregisterStatusProvider removes a status provider.
func (pa *PrinterAdapter) UnregisterStatusProvider(name string) {
	pa.mu.Lock()
	defer pa.mu.Unlock()
	delete(pa.statusProviders, name)
}

// SetGCodeExecutor sets the gcode executor function.
func (pa *PrinterAdapter) SetGCodeExecutor(executor func(script string) error) {
	pa.mu.Lock()
	defer pa.mu.Unlock()
	pa.gcodeExecutor = executor
}

// SetEmergencyStopHandler sets the emergency stop handler.
func (pa *PrinterAdapter) SetEmergencyStopHandler(handler func()) {
	pa.mu.Lock()
	defer pa.mu.Unlock()
	pa.emergencyStopHandler = handler
}

// SetKlippyStateGetter sets the klippy state getter function.
func (pa *PrinterAdapter) SetKlippyStateGetter(getter func() string) {
	pa.mu.Lock()
	defer pa.mu.Unlock()
	pa.klippyStateGetter = getter
}

// GetObjectsList implements PrinterInterface.
func (pa *PrinterAdapter) GetObjectsList() []string {
	pa.mu.RLock()
	defer pa.mu.RUnlock()

	objects := make([]string, 0, len(pa.statusProviders))
	for name := range pa.statusProviders {
		objects = append(objects, name)
	}
	return objects
}

// GetObjectStatus implements PrinterInterface.
func (pa *PrinterAdapter) GetObjectStatus(name string, attrs []string) map[string]any {
	pa.mu.RLock()
	provider, ok := pa.statusProviders[name]
	pa.mu.RUnlock()

	if !ok {
		return nil
	}

	return provider(attrs)
}

// ExecuteGCode implements PrinterInterface.
func (pa *PrinterAdapter) ExecuteGCode(script string) error {
	pa.mu.RLock()
	executor := pa.gcodeExecutor
	pa.mu.RUnlock()

	if executor != nil {
		// Split script by newlines and execute each line
		lines := strings.Split(script, "\n")
		for _, line := range lines {
			line = strings.TrimSpace(line)
			if line == "" || strings.HasPrefix(line, ";") {
				continue
			}
			if err := executor(line); err != nil {
				return err
			}
		}
	}
	return nil
}

// EmergencyStop implements PrinterInterface.
func (pa *PrinterAdapter) EmergencyStop() {
	pa.mu.RLock()
	handler := pa.emergencyStopHandler
	pa.mu.RUnlock()

	if handler != nil {
		handler()
	}
}

// GetKlippyState implements PrinterInterface.
func (pa *PrinterAdapter) GetKlippyState() string {
	pa.mu.RLock()
	getter := pa.klippyStateGetter
	pa.mu.RUnlock()

	if getter != nil {
		return getter()
	}
	return "ready"
}

// FilterStatus filters status map to only include requested attributes.
func FilterStatus(status map[string]any, attrs []string) map[string]any {
	if len(attrs) == 0 {
		return status
	}

	filtered := make(map[string]any)
	for _, attr := range attrs {
		if val, ok := status[attr]; ok {
			filtered[attr] = val
		}
	}
	return filtered
}
