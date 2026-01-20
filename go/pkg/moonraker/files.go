// File management API endpoints for Moonraker.
// Implements the file operations required by Fluidd/Mainsail.
package moonraker

import (
	"crypto/sha256"
	"encoding/hex"
	"encoding/json"
	"fmt"
	"io"
	"log"
	"net/http"
	"os"
	"path/filepath"
	"sort"
	"strings"
	"sync"
	"time"
)

// FileManager manages file operations for the Moonraker API.
type FileManager struct {
	// Root directories for different file types
	roots map[string]string // root name -> path

	// File metadata cache
	metadataCache map[string]*FileMetadata
	cacheMu       sync.RWMutex
}

// FileMetadata holds metadata for a file.
type FileMetadata struct {
	Path         string    `json:"path"`
	Filename     string    `json:"filename"`
	Modified     float64   `json:"modified"`
	Size         int64     `json:"size"`
	Permissions  string    `json:"permissions"`

	// GCode-specific metadata
	PrintStartTime *float64 `json:"print_start_time,omitempty"`
	JobID          *string  `json:"job_id,omitempty"`
	Slicer         string   `json:"slicer,omitempty"`
	SlicerVersion  string   `json:"slicer_version,omitempty"`

	// Thumbnail info
	Thumbnails []Thumbnail `json:"thumbnails,omitempty"`

	// Print estimates
	EstimatedTime   *float64 `json:"estimated_time,omitempty"`
	FilamentTotal   *float64 `json:"filament_total,omitempty"`
	FilamentWeight  *float64 `json:"filament_weight,omitempty"`
	FirstLayerHeight *float64 `json:"first_layer_height,omitempty"`
	LayerHeight      *float64 `json:"layer_height,omitempty"`
	ObjectHeight     *float64 `json:"object_height,omitempty"`
}

// Thumbnail holds thumbnail information.
type Thumbnail struct {
	Width        int    `json:"width"`
	Height       int    `json:"height"`
	Size         int64  `json:"size"`
	RelativePath string `json:"relative_path"`
}

// FileItem represents a file or directory in a listing.
type FileItem struct {
	Path        string  `json:"path"`
	Modified    float64 `json:"modified"`
	Size        int64   `json:"size"`
	Permissions string  `json:"permissions"`
}

// DirItem represents a directory in a listing.
type DirItem struct {
	Dirname     string  `json:"dirname"`
	Modified    float64 `json:"modified"`
	Size        int64   `json:"size"`
	Permissions string  `json:"permissions"`
}

// NewFileManager creates a new file manager.
func NewFileManager() *FileManager {
	return &FileManager{
		roots: map[string]string{
			"gcodes":  "",
			"config":  "",
			"config_examples": "",
			"docs":    "",
		},
		metadataCache: make(map[string]*FileMetadata),
	}
}

// SetRoot sets the root directory for a file type.
func (fm *FileManager) SetRoot(name, path string) error {
	// Ensure directory exists
	if err := os.MkdirAll(path, 0755); err != nil {
		return fmt.Errorf("failed to create directory %s: %w", path, err)
	}
	fm.roots[name] = path
	return nil
}

// GetRoot returns the root directory for a file type.
func (fm *FileManager) GetRoot(name string) (string, error) {
	root, ok := fm.roots[name]
	if !ok {
		return "", fmt.Errorf("unknown root: %s", name)
	}
	if root == "" {
		return "", fmt.Errorf("root %s not configured", name)
	}
	return root, nil
}

// ListFiles lists files in a directory.
func (fm *FileManager) ListFiles(root, path string) ([]FileItem, []DirItem, error) {
	rootPath, err := fm.GetRoot(root)
	if err != nil {
		return nil, nil, err
	}

	fullPath := filepath.Join(rootPath, path)

	// Security check: ensure path is within root
	absPath, err := filepath.Abs(fullPath)
	if err != nil {
		return nil, nil, err
	}
	absRoot, err := filepath.Abs(rootPath)
	if err != nil {
		return nil, nil, err
	}
	if !strings.HasPrefix(absPath, absRoot) {
		return nil, nil, fmt.Errorf("path traversal detected")
	}

	entries, err := os.ReadDir(fullPath)
	if err != nil {
		return nil, nil, err
	}

	var files []FileItem
	var dirs []DirItem

	for _, entry := range entries {
		info, err := entry.Info()
		if err != nil {
			continue
		}

		modified := float64(info.ModTime().Unix()) + float64(info.ModTime().Nanosecond())/1e9
		perms := "rw"

		if entry.IsDir() {
			dirs = append(dirs, DirItem{
				Dirname:     entry.Name(),
				Modified:    modified,
				Size:        info.Size(),
				Permissions: perms,
			})
		} else {
			relativePath := path
			if relativePath != "" {
				relativePath = filepath.Join(relativePath, entry.Name())
			} else {
				relativePath = entry.Name()
			}
			files = append(files, FileItem{
				Path:        relativePath,
				Modified:    modified,
				Size:        info.Size(),
				Permissions: perms,
			})
		}
	}

	// Sort by name
	sort.Slice(files, func(i, j int) bool {
		return files[i].Path < files[j].Path
	})
	sort.Slice(dirs, func(i, j int) bool {
		return dirs[i].Dirname < dirs[j].Dirname
	})

	return files, dirs, nil
}

// GetFileMetadata returns metadata for a file.
func (fm *FileManager) GetFileMetadata(root, path string) (*FileMetadata, error) {
	rootPath, err := fm.GetRoot(root)
	if err != nil {
		return nil, err
	}

	fullPath := filepath.Join(rootPath, path)

	info, err := os.Stat(fullPath)
	if err != nil {
		return nil, err
	}

	meta := &FileMetadata{
		Path:        path,
		Filename:    filepath.Base(path),
		Modified:    float64(info.ModTime().Unix()) + float64(info.ModTime().Nanosecond())/1e9,
		Size:        info.Size(),
		Permissions: "rw",
	}

	// Parse GCode metadata if applicable
	if root == "gcodes" && strings.HasSuffix(strings.ToLower(path), ".gcode") {
		fm.parseGCodeMetadata(fullPath, meta)
	}

	return meta, nil
}

// parseGCodeMetadata extracts metadata from a GCode file header.
func (fm *FileManager) parseGCodeMetadata(path string, meta *FileMetadata) {
	file, err := os.Open(path)
	if err != nil {
		return
	}
	defer file.Close()

	// Read first 64KB for metadata
	buf := make([]byte, 64*1024)
	n, _ := file.Read(buf)
	if n == 0 {
		return
	}
	header := string(buf[:n])

	// Parse common slicer comments
	lines := strings.Split(header, "\n")
	for _, line := range lines {
		line = strings.TrimSpace(line)
		if !strings.HasPrefix(line, ";") {
			continue
		}
		line = strings.TrimPrefix(line, ";")
		line = strings.TrimSpace(line)

		// PrusaSlicer/SuperSlicer style
		if strings.HasPrefix(line, "generated by ") {
			parts := strings.SplitN(line, " ", 4)
			if len(parts) >= 3 {
				meta.Slicer = parts[2]
			}
			if len(parts) >= 4 {
				meta.SlicerVersion = strings.TrimPrefix(parts[3], "on ")
			}
		}

		// Cura style
		if strings.HasPrefix(line, "FLAVOR:") {
			meta.Slicer = "Cura"
		}

		// Parse numeric values
		if strings.Contains(line, "estimated printing time") {
			// Parse time estimate
		}
		if strings.Contains(line, "filament used") {
			// Parse filament usage
		}
		if strings.HasPrefix(line, "layer_height") {
			var height float64
			if _, err := fmt.Sscanf(line, "layer_height = %f", &height); err == nil {
				meta.LayerHeight = &height
			}
		}
		if strings.HasPrefix(line, "first_layer_height") {
			var height float64
			if _, err := fmt.Sscanf(line, "first_layer_height = %f", &height); err == nil {
				meta.FirstLayerHeight = &height
			}
		}
	}
}

// UploadFile uploads a file.
func (fm *FileManager) UploadFile(root, path string, data io.Reader, checkExists bool) (*FileMetadata, error) {
	rootPath, err := fm.GetRoot(root)
	if err != nil {
		return nil, err
	}

	fullPath := filepath.Join(rootPath, path)

	// Security check
	absPath, err := filepath.Abs(fullPath)
	if err != nil {
		return nil, err
	}
	absRoot, err := filepath.Abs(rootPath)
	if err != nil {
		return nil, err
	}
	if !strings.HasPrefix(absPath, absRoot) {
		return nil, fmt.Errorf("path traversal detected")
	}

	// Check if file exists
	if checkExists {
		if _, err := os.Stat(fullPath); err == nil {
			return nil, fmt.Errorf("file already exists: %s", path)
		}
	}

	// Create parent directory
	if err := os.MkdirAll(filepath.Dir(fullPath), 0755); err != nil {
		return nil, err
	}

	// Create file
	file, err := os.Create(fullPath)
	if err != nil {
		return nil, err
	}
	defer file.Close()

	// Copy data
	if _, err := io.Copy(file, data); err != nil {
		os.Remove(fullPath)
		return nil, err
	}

	// Return metadata
	return fm.GetFileMetadata(root, path)
}

// DeleteFile deletes a file.
func (fm *FileManager) DeleteFile(root, path string) error {
	rootPath, err := fm.GetRoot(root)
	if err != nil {
		return err
	}

	fullPath := filepath.Join(rootPath, path)

	// Security check
	absPath, err := filepath.Abs(fullPath)
	if err != nil {
		return err
	}
	absRoot, err := filepath.Abs(rootPath)
	if err != nil {
		return err
	}
	if !strings.HasPrefix(absPath, absRoot) {
		return fmt.Errorf("path traversal detected")
	}

	return os.Remove(fullPath)
}

// MoveFile moves/renames a file.
func (fm *FileManager) MoveFile(root, srcPath, dstPath string) error {
	rootPath, err := fm.GetRoot(root)
	if err != nil {
		return err
	}

	srcFull := filepath.Join(rootPath, srcPath)
	dstFull := filepath.Join(rootPath, dstPath)

	// Security checks
	absSrc, err := filepath.Abs(srcFull)
	if err != nil {
		return err
	}
	absDst, err := filepath.Abs(dstFull)
	if err != nil {
		return err
	}
	absRoot, err := filepath.Abs(rootPath)
	if err != nil {
		return err
	}
	if !strings.HasPrefix(absSrc, absRoot) || !strings.HasPrefix(absDst, absRoot) {
		return fmt.Errorf("path traversal detected")
	}

	// Create destination directory
	if err := os.MkdirAll(filepath.Dir(dstFull), 0755); err != nil {
		return err
	}

	return os.Rename(srcFull, dstFull)
}

// CopyFile copies a file.
func (fm *FileManager) CopyFile(root, srcPath, dstPath string) error {
	rootPath, err := fm.GetRoot(root)
	if err != nil {
		return err
	}

	srcFull := filepath.Join(rootPath, srcPath)
	dstFull := filepath.Join(rootPath, dstPath)

	// Security checks
	absSrc, err := filepath.Abs(srcFull)
	if err != nil {
		return err
	}
	absDst, err := filepath.Abs(dstFull)
	if err != nil {
		return err
	}
	absRoot, err := filepath.Abs(rootPath)
	if err != nil {
		return err
	}
	if !strings.HasPrefix(absSrc, absRoot) || !strings.HasPrefix(absDst, absRoot) {
		return fmt.Errorf("path traversal detected")
	}

	// Open source
	src, err := os.Open(srcFull)
	if err != nil {
		return err
	}
	defer src.Close()

	// Create destination directory
	if err := os.MkdirAll(filepath.Dir(dstFull), 0755); err != nil {
		return err
	}

	// Create destination
	dst, err := os.Create(dstFull)
	if err != nil {
		return err
	}
	defer dst.Close()

	_, err = io.Copy(dst, src)
	return err
}

// CreateDirectory creates a directory.
func (fm *FileManager) CreateDirectory(root, path string) error {
	rootPath, err := fm.GetRoot(root)
	if err != nil {
		return err
	}

	fullPath := filepath.Join(rootPath, path)

	// Security check
	absPath, err := filepath.Abs(fullPath)
	if err != nil {
		return err
	}
	absRoot, err := filepath.Abs(rootPath)
	if err != nil {
		return err
	}
	if !strings.HasPrefix(absPath, absRoot) {
		return fmt.Errorf("path traversal detected")
	}

	return os.MkdirAll(fullPath, 0755)
}

// DeleteDirectory deletes a directory.
func (fm *FileManager) DeleteDirectory(root, path string, force bool) error {
	rootPath, err := fm.GetRoot(root)
	if err != nil {
		return err
	}

	fullPath := filepath.Join(rootPath, path)

	// Security check
	absPath, err := filepath.Abs(fullPath)
	if err != nil {
		return err
	}
	absRoot, err := filepath.Abs(rootPath)
	if err != nil {
		return err
	}
	if !strings.HasPrefix(absPath, absRoot) {
		return fmt.Errorf("path traversal detected")
	}

	// Don't allow deleting root
	if absPath == absRoot {
		return fmt.Errorf("cannot delete root directory")
	}

	if force {
		return os.RemoveAll(fullPath)
	}
	return os.Remove(fullPath)
}

// GetChecksum returns the checksum of a file.
func (fm *FileManager) GetChecksum(root, path string) (string, error) {
	rootPath, err := fm.GetRoot(root)
	if err != nil {
		return "", err
	}

	fullPath := filepath.Join(rootPath, path)

	file, err := os.Open(fullPath)
	if err != nil {
		return "", err
	}
	defer file.Close()

	hash := sha256.New()
	if _, err := io.Copy(hash, file); err != nil {
		return "", err
	}

	return hex.EncodeToString(hash.Sum(nil)), nil
}

// RegisterFileEndpoints registers file management HTTP endpoints on a server.
func (fm *FileManager) RegisterFileEndpoints(mux *http.ServeMux) {
	// List files
	mux.HandleFunc("/server/files/list", fm.handleList)

	// File metadata
	mux.HandleFunc("/server/files/metadata", fm.handleMetadata)

	// Directory operations
	mux.HandleFunc("/server/files/directory", fm.handleDirectory)

	// File operations
	mux.HandleFunc("/server/files/move", fm.handleMove)
	mux.HandleFunc("/server/files/copy", fm.handleCopy)

	// Upload (multipart)
	mux.HandleFunc("/server/files/upload", fm.handleUpload)

	// Download/delete (with path in URL)
	mux.HandleFunc("/server/files/", fm.handleFile)

	// Roots
	mux.HandleFunc("/server/files/roots", fm.handleRoots)
}

func (fm *FileManager) handleList(w http.ResponseWriter, r *http.Request) {
	root := r.URL.Query().Get("root")
	if root == "" {
		root = "gcodes"
	}
	path := r.URL.Query().Get("path")

	files, dirs, err := fm.ListFiles(root, path)
	if err != nil {
		writeJSONError(w, err, http.StatusBadRequest)
		return
	}

	// Get disk usage
	rootPath, _ := fm.GetRoot(root)
	var diskUsage map[string]any
	if rootPath != "" {
		// Simple disk usage (would need syscall for accurate info)
		diskUsage = map[string]any{
			"total": 0,
			"used":  0,
			"free":  0,
		}
	}

	writeJSON(w, map[string]any{
		"result": map[string]any{
			"dirs":       dirs,
			"files":      files,
			"disk_usage": diskUsage,
			"root_info": map[string]any{
				"name":        root,
				"permissions": "rw",
			},
		},
	})
}

func (fm *FileManager) handleMetadata(w http.ResponseWriter, r *http.Request) {
	path := r.URL.Query().Get("filename")
	if path == "" {
		writeJSONError(w, fmt.Errorf("missing filename parameter"), http.StatusBadRequest)
		return
	}

	meta, err := fm.GetFileMetadata("gcodes", path)
	if err != nil {
		writeJSONError(w, err, http.StatusNotFound)
		return
	}

	writeJSON(w, map[string]any{"result": meta})
}

func (fm *FileManager) handleDirectory(w http.ResponseWriter, r *http.Request) {
	root := r.URL.Query().Get("root")
	if root == "" {
		root = "gcodes"
	}
	path := r.URL.Query().Get("path")

	switch r.Method {
	case http.MethodGet:
		// List directory
		files, dirs, err := fm.ListFiles(root, path)
		if err != nil {
			writeJSONError(w, err, http.StatusBadRequest)
			return
		}
		writeJSON(w, map[string]any{
			"result": map[string]any{
				"dirs":  dirs,
				"files": files,
			},
		})

	case http.MethodPost:
		// Create directory
		if err := fm.CreateDirectory(root, path); err != nil {
			writeJSONError(w, err, http.StatusBadRequest)
			return
		}
		writeJSON(w, map[string]any{
			"result": map[string]any{
				"item": map[string]any{
					"path":        path,
					"root":        root,
					"modified":    float64(time.Now().Unix()),
					"size":        0,
					"permissions": "rw",
				},
				"action": "create_dir",
			},
		})

	case http.MethodDelete:
		// Delete directory
		force := r.URL.Query().Get("force") == "true"
		if err := fm.DeleteDirectory(root, path, force); err != nil {
			writeJSONError(w, err, http.StatusBadRequest)
			return
		}
		writeJSON(w, map[string]any{
			"result": map[string]any{
				"item": map[string]any{
					"path": path,
					"root": root,
				},
				"action": "delete_dir",
			},
		})

	default:
		http.Error(w, "Method not allowed", http.StatusMethodNotAllowed)
	}
}

func (fm *FileManager) handleMove(w http.ResponseWriter, r *http.Request) {
	if r.Method != http.MethodPost {
		http.Error(w, "Method not allowed", http.StatusMethodNotAllowed)
		return
	}

	var req struct {
		Source string `json:"source"`
		Dest   string `json:"dest"`
	}
	if err := json.NewDecoder(r.Body).Decode(&req); err != nil {
		writeJSONError(w, err, http.StatusBadRequest)
		return
	}

	// Extract root from source path
	root := "gcodes"
	srcPath := req.Source
	dstPath := req.Dest

	if err := fm.MoveFile(root, srcPath, dstPath); err != nil {
		writeJSONError(w, err, http.StatusBadRequest)
		return
	}

	writeJSON(w, map[string]any{
		"result": map[string]any{
			"item": map[string]any{
				"path": dstPath,
				"root": root,
			},
			"source_item": map[string]any{
				"path": srcPath,
				"root": root,
			},
			"action": "move_file",
		},
	})
}

func (fm *FileManager) handleCopy(w http.ResponseWriter, r *http.Request) {
	if r.Method != http.MethodPost {
		http.Error(w, "Method not allowed", http.StatusMethodNotAllowed)
		return
	}

	var req struct {
		Source string `json:"source"`
		Dest   string `json:"dest"`
	}
	if err := json.NewDecoder(r.Body).Decode(&req); err != nil {
		writeJSONError(w, err, http.StatusBadRequest)
		return
	}

	root := "gcodes"
	srcPath := req.Source
	dstPath := req.Dest

	if err := fm.CopyFile(root, srcPath, dstPath); err != nil {
		writeJSONError(w, err, http.StatusBadRequest)
		return
	}

	writeJSON(w, map[string]any{
		"result": map[string]any{
			"item": map[string]any{
				"path": dstPath,
				"root": root,
			},
			"action": "copy_file",
		},
	})
}

func (fm *FileManager) handleUpload(w http.ResponseWriter, r *http.Request) {
	if r.Method != http.MethodPost {
		http.Error(w, "Method not allowed", http.StatusMethodNotAllowed)
		return
	}

	// Parse multipart form
	if err := r.ParseMultipartForm(100 << 20); err != nil { // 100MB max
		writeJSONError(w, err, http.StatusBadRequest)
		return
	}

	file, header, err := r.FormFile("file")
	if err != nil {
		writeJSONError(w, err, http.StatusBadRequest)
		return
	}
	defer file.Close()

	root := r.FormValue("root")
	if root == "" {
		root = "gcodes"
	}
	path := r.FormValue("path")
	if path == "" {
		path = header.Filename
	}

	checkExists := r.FormValue("checksum") != ""

	meta, err := fm.UploadFile(root, path, file, checkExists)
	if err != nil {
		writeJSONError(w, err, http.StatusBadRequest)
		return
	}

	writeJSON(w, map[string]any{
		"result": map[string]any{
			"item":    meta,
			"root":    root,
			"action":  "create_file",
			"print_started": false,
		},
	})
}

func (fm *FileManager) handleFile(w http.ResponseWriter, r *http.Request) {
	// Extract path from URL: /server/files/{root}/{path...}
	urlPath := strings.TrimPrefix(r.URL.Path, "/server/files/")
	parts := strings.SplitN(urlPath, "/", 2)
	if len(parts) < 2 {
		http.Error(w, "Invalid path", http.StatusBadRequest)
		return
	}
	root := parts[0]
	path := parts[1]

	switch r.Method {
	case http.MethodGet:
		// Download file
		rootPath, err := fm.GetRoot(root)
		if err != nil {
			writeJSONError(w, err, http.StatusBadRequest)
			return
		}
		fullPath := filepath.Join(rootPath, path)
		http.ServeFile(w, r, fullPath)

	case http.MethodDelete:
		// Delete file
		if err := fm.DeleteFile(root, path); err != nil {
			writeJSONError(w, err, http.StatusBadRequest)
			return
		}
		writeJSON(w, map[string]any{
			"result": map[string]any{
				"item": map[string]any{
					"path": path,
					"root": root,
				},
				"action": "delete_file",
			},
		})

	default:
		http.Error(w, "Method not allowed", http.StatusMethodNotAllowed)
	}
}

func (fm *FileManager) handleRoots(w http.ResponseWriter, r *http.Request) {
	var roots []map[string]any
	for name, path := range fm.roots {
		if path == "" {
			continue
		}
		roots = append(roots, map[string]any{
			"name":        name,
			"path":        path,
			"permissions": "rw",
		})
	}
	writeJSON(w, map[string]any{"result": roots})
}

// Helper functions

func writeJSON(w http.ResponseWriter, data any) {
	w.Header().Set("Content-Type", "application/json")
	json.NewEncoder(w).Encode(data)
}

func writeJSONError(w http.ResponseWriter, err error, status int) {
	w.Header().Set("Content-Type", "application/json")
	w.WriteHeader(status)
	json.NewEncoder(w).Encode(map[string]any{
		"error": map[string]any{
			"code":    -32000,
			"message": err.Error(),
		},
	})
	log.Printf("Moonraker file error: %v", err)
}
