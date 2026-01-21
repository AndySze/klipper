// Query endstops - port of klippy/extras/query_endstops.py
//
// Utility for querying the current state of all endstops
//
// Copyright (C) 2018-2019 Kevin O'Connor <kevin@koconnor.net>
// Copyright (C) 2025 Go port
//
// This file may be distributed under the terms of the GNU GPLv3 license.

package hosth4

import (
	"fmt"
	"strings"
	"sync"
)

// endstopEntry holds a registered endstop and its name.
type endstopEntry struct {
	endstop *endstop
	name    string
}

// QueryEndstops provides endstop query functionality.
type QueryEndstops struct {
	rt *runtime

	endstops  []endstopEntry
	lastState map[string]bool // name -> triggered state
	mu        sync.RWMutex
}

// newQueryEndstops creates a new query endstops instance.
func newQueryEndstops(rt *runtime) *QueryEndstops {
	return &QueryEndstops{
		rt:        rt,
		endstops:  make([]endstopEntry, 0),
		lastState: make(map[string]bool),
	}
}

// RegisterEndstop registers an endstop for querying.
func (qe *QueryEndstops) RegisterEndstop(e *endstop, name string) {
	qe.mu.Lock()
	defer qe.mu.Unlock()
	qe.endstops = append(qe.endstops, endstopEntry{
		endstop: e,
		name:    name,
	})
}

// QueryAll queries all registered endstops and updates last state.
func (qe *QueryEndstops) QueryAll() (map[string]bool, error) {
	if qe.rt == nil || qe.rt.toolhead == nil {
		return nil, fmt.Errorf("toolhead not available")
	}

	// Get print time
	printTime, err := qe.rt.toolhead.getLastMoveTime()
	if err != nil {
		return nil, fmt.Errorf("get print time: %w", err)
	}

	qe.mu.Lock()
	defer qe.mu.Unlock()

	// Query each endstop
	results := make(map[string]bool)
	for _, entry := range qe.endstops {
		triggered, err := entry.endstop.QueryEndstop(printTime)
		if err != nil {
			// Log error but continue querying other endstops
			results[entry.name] = false
			continue
		}
		results[entry.name] = triggered
	}

	// Update last state
	qe.lastState = results
	return results, nil
}

// GetLastState returns the last queried state.
func (qe *QueryEndstops) GetLastState() map[string]bool {
	qe.mu.RLock()
	defer qe.mu.RUnlock()
	result := make(map[string]bool)
	for k, v := range qe.lastState {
		result[k] = v
	}
	return result
}

// GetStatus returns the query endstops status.
func (qe *QueryEndstops) GetStatus() map[string]any {
	qe.mu.RLock()
	defer qe.mu.RUnlock()
	lastQuery := make(map[string]bool)
	for k, v := range qe.lastState {
		lastQuery[k] = v
	}
	return map[string]any{
		"last_query": lastQuery,
	}
}

// cmdQueryEndstops handles the QUERY_ENDSTOPS / M119 command.
func (qe *QueryEndstops) cmdQueryEndstops() (string, error) {
	results, err := qe.QueryAll()
	if err != nil {
		return "", err
	}

	// Build response message
	var parts []string
	for name, triggered := range results {
		state := "open"
		if triggered {
			state = "TRIGGERED"
		}
		parts = append(parts, fmt.Sprintf("%s:%s", name, state))
	}

	return strings.Join(parts, " "), nil
}

// handleWebRequest handles the query_endstops/status webhook.
func (qe *QueryEndstops) handleWebRequest() (map[string]string, error) {
	results, err := qe.QueryAll()
	if err != nil {
		return nil, err
	}

	// Convert to string map
	response := make(map[string]string)
	for name, triggered := range results {
		if triggered {
			response[name] = "TRIGGERED"
		} else {
			response[name] = "open"
		}
	}
	return response, nil
}

// GetEndstopNames returns the names of all registered endstops.
func (qe *QueryEndstops) GetEndstopNames() []string {
	qe.mu.RLock()
	defer qe.mu.RUnlock()
	names := make([]string, len(qe.endstops))
	for i, entry := range qe.endstops {
		names[i] = entry.name
	}
	return names
}
