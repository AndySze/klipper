// Print history API endpoints for Moonraker.
// Tracks print jobs and their status.
package moonraker

import (
	"crypto/rand"
	"encoding/hex"
	"fmt"
	"net/http"
	"sort"
	"sync"
	"time"
)

// HistoryManager manages print job history.
type HistoryManager struct {
	mu   sync.RWMutex
	jobs map[string]*PrintJob
	// Most recent jobs first
	jobOrder []string

	// Current active job
	activeJobID string
}

// PrintJob represents a print job record.
type PrintJob struct {
	JobID          string  `json:"job_id"`
	Exists         bool    `json:"exists"`
	EndTime        *float64 `json:"end_time"`
	FilamentUsed   float64 `json:"filament_used"`
	Filename       string  `json:"filename"`
	Metadata       map[string]any `json:"metadata"`
	PrintDuration  float64 `json:"print_duration"`
	Status         string  `json:"status"` // "in_progress", "completed", "cancelled", "error"
	StartTime      float64 `json:"start_time"`
	TotalDuration  float64 `json:"total_duration"`
}

// JobTotals holds aggregated job statistics.
type JobTotals struct {
	TotalJobs        int     `json:"total_jobs"`
	TotalTime        float64 `json:"total_time"`
	TotalPrintTime   float64 `json:"total_print_time"`
	TotalFilamentUsed float64 `json:"total_filament_used"`
	LongestJob       float64 `json:"longest_job"`
	LongestPrint     float64 `json:"longest_print"`
}

// NewHistoryManager creates a new history manager.
func NewHistoryManager() *HistoryManager {
	return &HistoryManager{
		jobs:     make(map[string]*PrintJob),
		jobOrder: make([]string, 0),
	}
}

// generateJobID generates a unique job ID.
func generateJobID() string {
	b := make([]byte, 6)
	rand.Read(b)
	return hex.EncodeToString(b)
}

// StartJob creates a new print job.
func (hm *HistoryManager) StartJob(filename string, metadata map[string]any) *PrintJob {
	hm.mu.Lock()
	defer hm.mu.Unlock()

	jobID := generateJobID()
	now := float64(time.Now().Unix())

	job := &PrintJob{
		JobID:     jobID,
		Exists:    true,
		Filename:  filename,
		Metadata:  metadata,
		Status:    "in_progress",
		StartTime: now,
	}

	hm.jobs[jobID] = job
	hm.jobOrder = append([]string{jobID}, hm.jobOrder...)
	hm.activeJobID = jobID

	return job
}

// UpdateJob updates the current active job.
func (hm *HistoryManager) UpdateJob(printDuration, filamentUsed float64) {
	hm.mu.Lock()
	defer hm.mu.Unlock()

	if hm.activeJobID == "" {
		return
	}

	job, ok := hm.jobs[hm.activeJobID]
	if !ok {
		return
	}

	job.PrintDuration = printDuration
	job.FilamentUsed = filamentUsed
	now := float64(time.Now().Unix())
	job.TotalDuration = now - job.StartTime
}

// FinishJob marks a job as finished.
func (hm *HistoryManager) FinishJob(status string) *PrintJob {
	hm.mu.Lock()
	defer hm.mu.Unlock()

	if hm.activeJobID == "" {
		return nil
	}

	job, ok := hm.jobs[hm.activeJobID]
	if !ok {
		return nil
	}

	now := float64(time.Now().Unix())
	job.EndTime = &now
	job.Status = status
	job.TotalDuration = now - job.StartTime

	hm.activeJobID = ""
	return job
}

// GetJob returns a job by ID.
func (hm *HistoryManager) GetJob(jobID string) (*PrintJob, error) {
	hm.mu.RLock()
	defer hm.mu.RUnlock()

	job, ok := hm.jobs[jobID]
	if !ok {
		return nil, fmt.Errorf("job not found: %s", jobID)
	}
	return job, nil
}

// ListJobs returns jobs with optional filtering and pagination.
func (hm *HistoryManager) ListJobs(limit, start int, since, before float64, order string) []*PrintJob {
	hm.mu.RLock()
	defer hm.mu.RUnlock()

	// Collect matching jobs
	var matches []*PrintJob
	for _, jobID := range hm.jobOrder {
		job := hm.jobs[jobID]
		if job == nil {
			continue
		}

		// Filter by time
		if since > 0 && job.StartTime < since {
			continue
		}
		if before > 0 && job.StartTime > before {
			continue
		}

		matches = append(matches, job)
	}

	// Sort
	if order == "asc" {
		sort.Slice(matches, func(i, j int) bool {
			return matches[i].StartTime < matches[j].StartTime
		})
	}
	// Default is desc (most recent first), which is already the order

	// Pagination
	if start > 0 && start < len(matches) {
		matches = matches[start:]
	} else if start >= len(matches) {
		matches = nil
	}

	if limit > 0 && limit < len(matches) {
		matches = matches[:limit]
	}

	return matches
}

// GetTotals returns aggregated job statistics.
func (hm *HistoryManager) GetTotals() *JobTotals {
	hm.mu.RLock()
	defer hm.mu.RUnlock()

	totals := &JobTotals{}

	for _, job := range hm.jobs {
		totals.TotalJobs++
		totals.TotalTime += job.TotalDuration
		totals.TotalPrintTime += job.PrintDuration
		totals.TotalFilamentUsed += job.FilamentUsed

		if job.TotalDuration > totals.LongestJob {
			totals.LongestJob = job.TotalDuration
		}
		if job.PrintDuration > totals.LongestPrint {
			totals.LongestPrint = job.PrintDuration
		}
	}

	return totals
}

// DeleteJob deletes a job from history.
func (hm *HistoryManager) DeleteJob(jobID string) error {
	hm.mu.Lock()
	defer hm.mu.Unlock()

	if _, ok := hm.jobs[jobID]; !ok {
		return fmt.Errorf("job not found: %s", jobID)
	}

	delete(hm.jobs, jobID)

	// Remove from order
	for i, id := range hm.jobOrder {
		if id == jobID {
			hm.jobOrder = append(hm.jobOrder[:i], hm.jobOrder[i+1:]...)
			break
		}
	}

	return nil
}

// ResetTotals clears all job history.
func (hm *HistoryManager) ResetTotals() {
	hm.mu.Lock()
	defer hm.mu.Unlock()

	hm.jobs = make(map[string]*PrintJob)
	hm.jobOrder = make([]string, 0)
	hm.activeJobID = ""
}

// RegisterHistoryEndpoints registers history HTTP endpoints.
func (hm *HistoryManager) RegisterHistoryEndpoints(mux *http.ServeMux) {
	mux.HandleFunc("/server/history/list", hm.handleList)
	mux.HandleFunc("/server/history/status", hm.handleStatus)
	mux.HandleFunc("/server/history/totals", hm.handleTotals)
	mux.HandleFunc("/server/history/job", hm.handleJob)
	mux.HandleFunc("/server/history/reset_totals", hm.handleResetTotals)
}

func (hm *HistoryManager) handleList(w http.ResponseWriter, r *http.Request) {
	q := r.URL.Query()

	limit := 50
	if l := q.Get("limit"); l != "" {
		fmt.Sscanf(l, "%d", &limit)
	}

	start := 0
	if s := q.Get("start"); s != "" {
		fmt.Sscanf(s, "%d", &start)
	}

	var since, before float64
	if s := q.Get("since"); s != "" {
		fmt.Sscanf(s, "%f", &since)
	}
	if b := q.Get("before"); b != "" {
		fmt.Sscanf(b, "%f", &before)
	}

	order := q.Get("order")
	if order == "" {
		order = "desc"
	}

	jobs := hm.ListJobs(limit, start, since, before, order)

	hm.mu.RLock()
	count := len(hm.jobs)
	hm.mu.RUnlock()

	writeJSON(w, map[string]any{
		"result": map[string]any{
			"count": count,
			"jobs":  jobs,
		},
	})
}

func (hm *HistoryManager) handleStatus(w http.ResponseWriter, r *http.Request) {
	hm.mu.RLock()
	jobID := hm.activeJobID
	var job *PrintJob
	if jobID != "" {
		job = hm.jobs[jobID]
	}
	hm.mu.RUnlock()

	result := map[string]any{}
	if job != nil {
		result["job"] = job
	}

	writeJSON(w, map[string]any{"result": result})
}

func (hm *HistoryManager) handleTotals(w http.ResponseWriter, r *http.Request) {
	totals := hm.GetTotals()

	writeJSON(w, map[string]any{
		"result": map[string]any{
			"job_totals": totals,
		},
	})
}

func (hm *HistoryManager) handleJob(w http.ResponseWriter, r *http.Request) {
	uid := r.URL.Query().Get("uid")
	if uid == "" {
		writeJSONError(w, fmt.Errorf("missing uid parameter"), http.StatusBadRequest)
		return
	}

	switch r.Method {
	case http.MethodGet:
		job, err := hm.GetJob(uid)
		if err != nil {
			writeJSONError(w, err, http.StatusNotFound)
			return
		}
		writeJSON(w, map[string]any{"result": map[string]any{"job": job}})

	case http.MethodDelete:
		if err := hm.DeleteJob(uid); err != nil {
			writeJSONError(w, err, http.StatusNotFound)
			return
		}
		writeJSON(w, map[string]any{
			"result": map[string]any{
				"deleted_jobs": []string{uid},
			},
		})

	default:
		http.Error(w, "Method not allowed", http.StatusMethodNotAllowed)
	}
}

func (hm *HistoryManager) handleResetTotals(w http.ResponseWriter, r *http.Request) {
	if r.Method != http.MethodPost {
		http.Error(w, "Method not allowed", http.StatusMethodNotAllowed)
		return
	}

	hm.ResetTotals()

	writeJSON(w, map[string]any{
		"result": map[string]any{
			"last_totals": hm.GetTotals(),
		},
	})
}
