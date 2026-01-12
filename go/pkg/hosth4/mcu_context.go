package hosth4

import (
	"fmt"
	"os"

	"klipper-go-migration/pkg/chelper"
	"klipper-go-migration/pkg/protocol"
)

// mcuContext holds all runtime resources for a single MCU.
// This includes the serial queue, command queue, protocol formats, and timing info.
type mcuContext struct {
	name    string
	dict    *protocol.Dictionary
	formats map[string]*protocol.MessageFormat
	freq    float64

	// Serial and command queues
	sq     *chelper.SerialQueue
	cqMain *chelper.CommandQueue

	// Command IDs (from dictionary)
	queueStepID int32
	setDirID    int32

	// Output file for debugging (optional)
	outputFile *os.File

	// Is this the primary MCU (timing source)?
	isPrimary bool
}

// newMCUContext creates a new MCU context with all required resources.
// If outputPath is non-empty, output will be written to that file.
func newMCUContext(name string, dict *protocol.Dictionary, outputPath string, isPrimary bool) (*mcuContext, error) {
	mcuFreq, err := dictConfigFloat(dict, "CLOCK_FREQ")
	if err != nil {
		return nil, fmt.Errorf("MCU %s: %w", name, err)
	}

	formats, err := dict.BuildCommandFormats()
	if err != nil {
		return nil, fmt.Errorf("MCU %s: failed to build command formats: %w", name, err)
	}

	queueStepID, err := dictCommandTag(dict, "queue_step oid=%c interval=%u count=%hu add=%hi")
	if err != nil {
		return nil, fmt.Errorf("MCU %s: %w", name, err)
	}

	setDirID, err := dictCommandTag(dict, "set_next_step_dir oid=%c dir=%c")
	if err != nil {
		return nil, fmt.Errorf("MCU %s: %w", name, err)
	}

	// Create command queue
	cqMain := chelper.NewCommandQueue()
	if cqMain == nil {
		return nil, fmt.Errorf("MCU %s: failed to create command queue", name)
	}

	// Create serial queue
	var outputFile *os.File
	var sq *chelper.SerialQueue

	if outputPath != "" {
		outputFile, err = os.Create(outputPath)
		if err != nil {
			cqMain.Free()
			return nil, fmt.Errorf("MCU %s: failed to create output file: %w", name, err)
		}
		sq, err = chelper.NewSerialQueue(int(outputFile.Fd()), 'f', 0, "serialq "+name)
		if err != nil {
			outputFile.Close()
			cqMain.Free()
			return nil, fmt.Errorf("MCU %s: failed to create serial queue: %w", name, err)
		}
	} else {
		// No output file - create a null serial queue (fd=-1)
		sq, err = chelper.NewSerialQueue(-1, 'f', 0, "serialq "+name)
		if err != nil {
			cqMain.Free()
			return nil, fmt.Errorf("MCU %s: failed to create serial queue: %w", name, err)
		}
	}

	// Set clock estimate for simulation timing
	sq.SetClockEst(1e12, chelper.Monotonic(), 0, 0)

	return &mcuContext{
		name:        name,
		dict:        dict,
		formats:     formats,
		freq:        mcuFreq,
		sq:          sq,
		cqMain:      cqMain,
		queueStepID: queueStepID,
		setDirID:    setDirID,
		outputFile:  outputFile,
		isPrimary:   isPrimary,
	}, nil
}

// Close releases all resources associated with this MCU context.
func (ctx *mcuContext) Close() {
	if ctx.cqMain != nil {
		ctx.cqMain.Free()
		ctx.cqMain = nil
	}
	if ctx.sq != nil {
		ctx.sq.Free()
		ctx.sq = nil
	}
	if ctx.outputFile != nil {
		ctx.outputFile.Close()
		ctx.outputFile = nil
	}
}

// sendLine encodes and sends a command line to this MCU.
func (ctx *mcuContext) sendLine(line string, cq *chelper.CommandQueue, minClock, reqClock uint64) error {
	b, err := protocol.EncodeCommand(ctx.formats, line)
	if err != nil {
		return err
	}
	ctx.sq.Send(cq, b, minClock, reqClock)
	return nil
}

// sendLineMain sends a command to this MCU using the main command queue.
func (ctx *mcuContext) sendLineMain(line string, minClock, reqClock uint64) error {
	return ctx.sendLine(line, ctx.cqMain, minClock, reqClock)
}

// mcuContextMap holds all MCU contexts in a multi-MCU setup.
type mcuContextMap struct {
	contexts map[string]*mcuContext
	primary  *mcuContext
	order    []string // MCU names in registration order
}

// newMCUContextMap creates a new empty MCU context map.
func newMCUContextMap() *mcuContextMap {
	return &mcuContextMap{
		contexts: make(map[string]*mcuContext),
		order:    make([]string, 0),
	}
}

// Add adds an MCU context to the map.
// The first MCU added becomes the primary.
func (m *mcuContextMap) Add(ctx *mcuContext) {
	if m.primary == nil {
		m.primary = ctx
		ctx.isPrimary = true
	}
	m.contexts[ctx.name] = ctx
	m.order = append(m.order, ctx.name)
}

// Get returns the MCU context for the given name, or nil if not found.
func (m *mcuContextMap) Get(name string) *mcuContext {
	return m.contexts[name]
}

// GetPrimary returns the primary MCU context.
func (m *mcuContextMap) GetPrimary() *mcuContext {
	return m.primary
}

// Names returns all MCU names in registration order.
func (m *mcuContextMap) Names() []string {
	result := make([]string, len(m.order))
	copy(result, m.order)
	return result
}

// Close releases all MCU contexts.
func (m *mcuContextMap) Close() {
	for _, ctx := range m.contexts {
		ctx.Close()
	}
	m.contexts = nil
	m.primary = nil
	m.order = nil
}

// Count returns the number of MCU contexts.
func (m *mcuContextMap) Count() int {
	return len(m.contexts)
}
