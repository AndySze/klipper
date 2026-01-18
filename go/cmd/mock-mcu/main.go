// mock-mcu creates a comprehensive simulated MCU for testing the Go host.
// It supports the full Klipper MCU protocol including:
// - Handshake and identification
// - Clock synchronization
// - Stepper motor control
// - GPIO/PWM output
// - Analog input (temperature sensors)
// - Endstops
//
// Usage:
//
//	mock-mcu -socket /tmp/klipper_mcu [-trace]
package main

import (
	"bytes"
	"compress/zlib"
	"encoding/json"
	"flag"
	"fmt"
	"net"
	"os"
	"os/signal"
	"sync"
	"syscall"
	"time"
)

const (
	CLOCK_FREQ = 50000000 // 50 MHz

	// Command IDs (must match dictionary)
	CMD_IDENTIFY          = 1
	CMD_GET_CONFIG        = 2
	CMD_ALLOCATE_OIDS     = 4
	CMD_GET_CLOCK         = 5
	CMD_FINALIZE_CONFIG   = 7
	CMD_CONFIG_STEPPER    = 10
	CMD_QUEUE_STEP        = 11
	CMD_SET_NEXT_STEP_DIR = 12
	CMD_RESET_STEP_CLOCK  = 13
	CMD_STEPPER_GET_POS   = 14
	CMD_ENDSTOP_HOME      = 15
	CMD_CONFIG_DIGITAL_OUT = 20
	CMD_CONFIG_PWM_OUT    = 21
	CMD_SCHED_DIGITAL_OUT = 22
	CMD_SCHED_PWM_OUT     = 23
	CMD_CONFIG_ANALOG_IN  = 30
	CMD_QUERY_ANALOG_IN   = 31
	CMD_CONFIG_ENDSTOP    = 40
	CMD_ENDSTOP_QUERY     = 41

	// Response IDs
	RESP_IDENTIFY       = 0
	RESP_CONFIG         = 3
	RESP_CLOCK          = 6
	RESP_STEPPER_POS    = 16
	RESP_ENDSTOP_STATE  = 42
	RESP_ANALOG_IN      = 32
	RESP_SHUTDOWN       = 50
)

// MCU state
type MCUState struct {
	mu sync.Mutex

	startTime    time.Time
	isConfigured bool
	configCRC    uint32
	oidCount     int

	// Steppers: oid -> stepper state
	steppers map[int]*StepperState

	// Digital outputs: oid -> state
	digitalOuts map[int]*DigitalOutState

	// PWM outputs: oid -> state
	pwmOuts map[int]*PWMOutState

	// Analog inputs: oid -> state
	analogIns map[int]*AnalogInState

	// Endstops: oid -> state
	endstops map[int]*EndstopState
}

type StepperState struct {
	oid         int
	stepPin     int
	dirPin      int
	minStopInterval uint32
	invertStep  bool
	position    int64
	dir         int // 1 or -1
	nextStepClock uint64
}

type DigitalOutState struct {
	oid   int
	pin   int
	value bool
}

type PWMOutState struct {
	oid      int
	pin      int
	value    uint16
	maxValue uint16
}

type AnalogInState struct {
	oid         int
	pin         int
	sampleTime  uint32
	sampleCount int
	reportClock uint32
	reportTicks uint32
	minValue    uint16
	maxValue    uint16
}

type EndstopState struct {
	oid       int
	pin       int
	pullup    bool
	triggered bool
	homingOid int
}

func NewMCUState() *MCUState {
	return &MCUState{
		startTime:   time.Now(),
		steppers:    make(map[int]*StepperState),
		digitalOuts: make(map[int]*DigitalOutState),
		pwmOuts:     make(map[int]*PWMOutState),
		analogIns:   make(map[int]*AnalogInState),
		endstops:    make(map[int]*EndstopState),
	}
}

func (m *MCUState) GetClock() uint32 {
	elapsed := time.Since(m.startTime)
	return uint32(elapsed.Seconds() * CLOCK_FREQ)
}

// CRC16-CCITT checksum
func crc16ccitt(data []byte) uint16 {
	crc := uint16(0xFFFF)
	for _, b := range data {
		d := b ^ uint8(crc&0xff)
		d ^= d << 4
		crc = ((uint16(d) << 8) | (crc >> 8)) ^ uint16(d>>4) ^ (uint16(d) << 3)
	}
	return crc
}

// Encode a VLQ integer (signed, matching Klipper's format)
func encodeVLQ(out *[]byte, v int32) {
	if v >= 0x4000000 || v < -0x4000000 {
		*out = append(*out, byte((v>>28)&0x7f)|0x80)
	}
	if v >= 0x200000 || v < -0x200000 {
		*out = append(*out, byte((v>>21)&0x7f)|0x80)
	}
	if v >= 0x4000 || v < -0x4000 {
		*out = append(*out, byte((v>>14)&0x7f)|0x80)
	}
	if v >= 0x60 || v < -0x20 {
		*out = append(*out, byte((v>>7)&0x7f)|0x80)
	}
	*out = append(*out, byte(v&0x7f))
}

// Decode a VLQ integer
func decodeVLQ(data []byte, pos int) (int32, int) {
	if pos >= len(data) {
		return 0, pos
	}
	c := data[pos]
	pos++
	v := int32(c & 0x7f)
	if (c & 0x60) == 0x60 {
		v |= -0x20
	}
	for (c & 0x80) != 0 {
		if pos >= len(data) {
			break
		}
		c = data[pos]
		pos++
		v = (v << 7) | int32(c&0x7f)
	}
	return v, pos
}

// Build a message block
func buildMessage(seq byte, msgData []byte) []byte {
	msgLen := len(msgData) + 5
	msg := make([]byte, 0, msgLen)
	msg = append(msg, byte(msgLen))
	msg = append(msg, seq)
	msg = append(msg, msgData...)
	crc := crc16ccitt(msg)
	msg = append(msg, byte(crc>>8), byte(crc&0xff))
	msg = append(msg, 0x7e)
	return msg
}

// Mock dictionary with comprehensive command set
var mockDict = map[string]interface{}{
	"commands": map[string]int{
		"identify offset=%u count=%u":                                    CMD_IDENTIFY,
		"get_config":                                                     CMD_GET_CONFIG,
		"allocate_oids count=%c":                                         CMD_ALLOCATE_OIDS,
		"get_clock":                                                      CMD_GET_CLOCK,
		"finalize_config crc=%u":                                         CMD_FINALIZE_CONFIG,
		"config_stepper oid=%c step_pin=%c dir_pin=%c invert_step=%c":    CMD_CONFIG_STEPPER,
		"queue_step oid=%c interval=%u count=%hu add=%hi":                CMD_QUEUE_STEP,
		"set_next_step_dir oid=%c dir=%c":                                CMD_SET_NEXT_STEP_DIR,
		"reset_step_clock oid=%c clock=%u":                               CMD_RESET_STEP_CLOCK,
		"stepper_get_position oid=%c":                                    CMD_STEPPER_GET_POS,
		"endstop_home oid=%c clock=%u sample_ticks=%u sample_count=%c pin_value=%c trsync_oid=%c trigger_reason=%c": CMD_ENDSTOP_HOME,
		"config_digital_out oid=%c pin=%u value=%c default_value=%c max_duration=%u": CMD_CONFIG_DIGITAL_OUT,
		"config_pwm_out oid=%c pin=%u cycle_ticks=%u value=%hu default_value=%hu max_duration=%u": CMD_CONFIG_PWM_OUT,
		"schedule_digital_out oid=%c clock=%u value=%c":                  CMD_SCHED_DIGITAL_OUT,
		"schedule_pwm_out oid=%c clock=%u value=%hu":                     CMD_SCHED_PWM_OUT,
		"config_analog_in oid=%c pin=%u":                                 CMD_CONFIG_ANALOG_IN,
		"query_analog_in oid=%c clock=%u sample_ticks=%u sample_count=%c rest_ticks=%u min_value=%hu max_value=%hu range_check_count=%c": CMD_QUERY_ANALOG_IN,
		"config_endstop oid=%c pin=%c pull_up=%c":                        CMD_CONFIG_ENDSTOP,
		"endstop_query_state oid=%c":                                     CMD_ENDSTOP_QUERY,
	},
	"responses": map[string]int{
		"identify_response offset=%u data=%*s":                           RESP_IDENTIFY,
		"config is_config=%c crc=%u is_shutdown=%c move_count=%hu":       RESP_CONFIG,
		"clock clock=%u":                                                 RESP_CLOCK,
		"stepper_position oid=%c pos=%i":                                 RESP_STEPPER_POS,
		"endstop_state oid=%c homing=%c next_clock=%u pin_value=%c":      RESP_ENDSTOP_STATE,
		"analog_in_state oid=%c next_clock=%u value=%hu":                 RESP_ANALOG_IN,
		"shutdown clock=%u static_string_id=%hu":                         RESP_SHUTDOWN,
	},
	"config": map[string]interface{}{
		"CLOCK_FREQ":     CLOCK_FREQ,
		"MCU":            "mock-mcu",
		"BUILD_VERSIONS": "mock-mcu v1.0 gcc",
	},
	"enumerations": map[string]map[string]int{
		"pin": {
			"PA0": 0, "PA1": 1, "PA2": 2, "PA3": 3, "PA4": 4, "PA5": 5, "PA6": 6, "PA7": 7,
			"PB0": 10, "PB1": 11, "PB2": 12, "PB3": 13, "PB4": 14, "PB5": 15, "PB6": 16, "PB7": 17,
			"PC0": 20, "PC1": 21, "PC2": 22, "PC3": 23, "PC4": 24, "PC5": 25, "PC6": 26, "PC7": 27,
			"PD0": 30, "PD1": 31, "PD2": 32, "PD3": 33, "PD4": 34, "PD5": 35, "PD6": 36, "PD7": 37,
		},
	},
	"version": "mock-mcu-v1.0-test",
}

func main() {
	socketPath := flag.String("socket", "/tmp/klipper_mcu", "Unix socket path")
	trace := flag.Bool("trace", false, "Enable trace output")
	flag.Parse()

	os.Remove(*socketPath)

	listener, err := net.Listen("unix", *socketPath)
	if err != nil {
		fmt.Fprintf(os.Stderr, "Error creating socket: %v\n", err)
		os.Exit(1)
	}
	defer listener.Close()
	defer os.Remove(*socketPath)

	fmt.Printf("Mock MCU listening on %s\n", *socketPath)
	fmt.Printf("Clock frequency: %d Hz\n", CLOCK_FREQ)
	fmt.Println("Press Ctrl+C to stop")

	sigCh := make(chan os.Signal, 1)
	signal.Notify(sigCh, syscall.SIGINT, syscall.SIGTERM)

	connCh := make(chan net.Conn, 1)
	go func() {
		for {
			conn, err := listener.Accept()
			if err != nil {
				return
			}
			connCh <- conn
		}
	}()

	// Prepare zlib-compressed dictionary
	dictJSON, _ := json.Marshal(mockDict)
	var dictBuf bytes.Buffer
	zlibWriter := zlib.NewWriter(&dictBuf)
	zlibWriter.Write(dictJSON)
	zlibWriter.Close()
	dictCompressed := dictBuf.Bytes()
	fmt.Printf("Dictionary: %d bytes compressed (from %d)\n", len(dictCompressed), len(dictJSON))

	for {
		select {
		case <-sigCh:
			fmt.Println("\nShutting down...")
			return
		case conn := <-connCh:
			fmt.Println("Client connected")
			state := NewMCUState()
			go handleConnection(conn, dictCompressed, state, *trace)
		}
	}
}

func handleConnection(conn net.Conn, dictCompressed []byte, state *MCUState, trace bool) {
	defer conn.Close()

	buf := make([]byte, 256)
	seq := byte(0)

	// Background goroutine for periodic analog reports
	stopCh := make(chan struct{})
	defer close(stopCh)

	go func() {
		ticker := time.NewTicker(100 * time.Millisecond)
		defer ticker.Stop()
		for {
			select {
			case <-stopCh:
				return
			case <-ticker.C:
				state.mu.Lock()
				for _, ain := range state.analogIns {
					if ain.reportTicks > 0 {
						// Simulate temperature reading (around 25°C for a 10k thermistor)
						// ADC value around 512 for 25°C
						value := uint16(500 + (time.Now().UnixNano()%50)) // Slight variation
						if trace {
							fmt.Printf("  -> analog_in_state oid=%d value=%d\n", ain.oid, value)
						}
						var resp []byte
						encodeVLQ(&resp, RESP_ANALOG_IN)
						encodeVLQ(&resp, int32(ain.oid))
						encodeVLQ(&resp, int32(state.GetClock()+ain.reportTicks))
						encodeVLQ(&resp, int32(value))
						msg := buildMessage(seq|0x10, resp)
						conn.Write(msg)
						seq = (seq + 1) & 0x0f
					}
				}
				state.mu.Unlock()
			}
		}
	}()

	for {
		conn.SetReadDeadline(time.Now().Add(100 * time.Millisecond))
		n, err := conn.Read(buf)
		if err != nil {
			if netErr, ok := err.(net.Error); ok && netErr.Timeout() {
				continue
			}
			fmt.Printf("Client disconnected: %v\n", err)
			return
		}

		if n < 5 {
			continue
		}

		// Parse message
		msgLen := int(buf[0])
		if msgLen > n || msgLen < 5 {
			continue
		}

		// Verify CRC
		receivedCRC := uint16(buf[msgLen-3])<<8 | uint16(buf[msgLen-2])
		calculatedCRC := crc16ccitt(buf[:msgLen-3])
		if receivedCRC != calculatedCRC {
			if trace {
				fmt.Printf("CRC mismatch: got %04x, expected %04x\n", receivedCRC, calculatedCRC)
			}
			continue
		}

		if msgLen <= 5 {
			// ACK
			ack := buildMessage(seq|0x10, nil)
			conn.Write(ack)
			seq = (seq + 1) & 0x0f
			continue
		}

		cmdID, pos := decodeVLQ(buf[2:msgLen-3], 0)
		pos += 2

		if trace {
			fmt.Printf("CMD %d (len=%d) raw: %x\n", cmdID, msgLen, buf[:msgLen])
		}

		var resp []byte

		switch int(cmdID) {
		case CMD_IDENTIFY:
			offset, np := decodeVLQ(buf[pos:msgLen-3], 0)
			pos += np
			count, _ := decodeVLQ(buf[pos:msgLen-3], 0)
			if trace {
				fmt.Printf("  identify offset=%d count=%d\n", offset, count)
			}

			start := int(offset)
			if start > len(dictCompressed) {
				start = len(dictCompressed)
			}
			end := start + int(count)
			if end > len(dictCompressed) {
				end = len(dictCompressed)
			}
			chunk := dictCompressed[start:end]

			encodeVLQ(&resp, RESP_IDENTIFY)
			encodeVLQ(&resp, offset)
			resp = append(resp, byte(len(chunk)))
			resp = append(resp, chunk...)

		case CMD_GET_CONFIG:
			if trace {
				fmt.Println("  get_config")
			}
			state.mu.Lock()
			isConfig := byte(0)
			if state.isConfigured {
				isConfig = 1
			}
			crc := state.configCRC
			state.mu.Unlock()

			encodeVLQ(&resp, RESP_CONFIG)
			encodeVLQ(&resp, int32(isConfig))
			encodeVLQ(&resp, int32(crc))
			encodeVLQ(&resp, 0) // is_shutdown
			encodeVLQ(&resp, 256) // move_count

		case CMD_GET_CLOCK:
			clock := state.GetClock()
			if trace {
				fmt.Printf("  get_clock -> %d\n", clock)
			}
			encodeVLQ(&resp, RESP_CLOCK)
			encodeVLQ(&resp, int32(clock))

		case CMD_ALLOCATE_OIDS:
			count, _ := decodeVLQ(buf[pos:msgLen-3], 0)
			if trace {
				fmt.Printf("  allocate_oids count=%d\n", count)
			}
			state.mu.Lock()
			state.oidCount = int(count)
			state.mu.Unlock()
			// No response, just ACK

		case CMD_FINALIZE_CONFIG:
			crc, _ := decodeVLQ(buf[pos:msgLen-3], 0)
			if trace {
				fmt.Printf("  finalize_config crc=%d\n", crc)
			}
			state.mu.Lock()
			state.isConfigured = true
			state.configCRC = uint32(crc)
			state.mu.Unlock()
			// No response, just ACK

		case CMD_CONFIG_STEPPER:
			oid, np := decodeVLQ(buf[pos:msgLen-3], 0)
			pos += np
			stepPin, np := decodeVLQ(buf[pos:msgLen-3], 0)
			pos += np
			dirPin, np := decodeVLQ(buf[pos:msgLen-3], 0)
			pos += np
			invertStep, _ := decodeVLQ(buf[pos:msgLen-3], 0)
			if trace {
				fmt.Printf("  config_stepper oid=%d step_pin=%d dir_pin=%d invert=%d\n",
					oid, stepPin, dirPin, invertStep)
			}
			state.mu.Lock()
			state.steppers[int(oid)] = &StepperState{
				oid:        int(oid),
				stepPin:    int(stepPin),
				dirPin:     int(dirPin),
				invertStep: invertStep != 0,
				dir:        1,
			}
			state.mu.Unlock()

		case CMD_QUEUE_STEP:
			oid, np := decodeVLQ(buf[pos:msgLen-3], 0)
			pos += np
			interval, np := decodeVLQ(buf[pos:msgLen-3], 0)
			pos += np
			count, np := decodeVLQ(buf[pos:msgLen-3], 0)
			pos += np
			add, _ := decodeVLQ(buf[pos:msgLen-3], 0)
			if trace {
				fmt.Printf("  queue_step oid=%d interval=%d count=%d add=%d\n",
					oid, interval, count, add)
			}
			state.mu.Lock()
			if s, ok := state.steppers[int(oid)]; ok {
				// Simulate step execution
				s.position += int64(count) * int64(s.dir)
			}
			state.mu.Unlock()

		case CMD_SET_NEXT_STEP_DIR:
			oid, np := decodeVLQ(buf[pos:msgLen-3], 0)
			pos += np
			dir, _ := decodeVLQ(buf[pos:msgLen-3], 0)
			if trace {
				fmt.Printf("  set_next_step_dir oid=%d dir=%d\n", oid, dir)
			}
			state.mu.Lock()
			if s, ok := state.steppers[int(oid)]; ok {
				if dir == 0 {
					s.dir = -1
				} else {
					s.dir = 1
				}
			}
			state.mu.Unlock()

		case CMD_RESET_STEP_CLOCK:
			oid, np := decodeVLQ(buf[pos:msgLen-3], 0)
			pos += np
			clock, _ := decodeVLQ(buf[pos:msgLen-3], 0)
			if trace {
				fmt.Printf("  reset_step_clock oid=%d clock=%d\n", oid, clock)
			}
			state.mu.Lock()
			if s, ok := state.steppers[int(oid)]; ok {
				s.nextStepClock = uint64(clock)
			}
			state.mu.Unlock()

		case CMD_STEPPER_GET_POS:
			oid, _ := decodeVLQ(buf[pos:msgLen-3], 0)
			state.mu.Lock()
			pos := int64(0)
			if s, ok := state.steppers[int(oid)]; ok {
				pos = s.position
			}
			state.mu.Unlock()
			if trace {
				fmt.Printf("  stepper_get_position oid=%d -> %d\n", oid, pos)
			}
			encodeVLQ(&resp, RESP_STEPPER_POS)
			encodeVLQ(&resp, oid)
			encodeVLQ(&resp, int32(pos))

		case CMD_CONFIG_DIGITAL_OUT:
			oid, np := decodeVLQ(buf[pos:msgLen-3], 0)
			pos += np
			pin, np := decodeVLQ(buf[pos:msgLen-3], 0)
			pos += np
			value, np := decodeVLQ(buf[pos:msgLen-3], 0)
			pos += np
			defaultVal, _ := decodeVLQ(buf[pos:msgLen-3], 0)
			if trace {
				fmt.Printf("  config_digital_out oid=%d pin=%d value=%d default=%d\n",
					oid, pin, value, defaultVal)
			}
			state.mu.Lock()
			state.digitalOuts[int(oid)] = &DigitalOutState{
				oid:   int(oid),
				pin:   int(pin),
				value: value != 0,
			}
			state.mu.Unlock()

		case CMD_SCHED_DIGITAL_OUT:
			oid, np := decodeVLQ(buf[pos:msgLen-3], 0)
			pos += np
			clock, np := decodeVLQ(buf[pos:msgLen-3], 0)
			pos += np
			value, _ := decodeVLQ(buf[pos:msgLen-3], 0)
			if trace {
				fmt.Printf("  schedule_digital_out oid=%d clock=%d value=%d\n", oid, clock, value)
			}
			state.mu.Lock()
			if d, ok := state.digitalOuts[int(oid)]; ok {
				d.value = value != 0
			}
			state.mu.Unlock()

		case CMD_CONFIG_PWM_OUT:
			oid, np := decodeVLQ(buf[pos:msgLen-3], 0)
			pos += np
			pin, np := decodeVLQ(buf[pos:msgLen-3], 0)
			pos += np
			_, np = decodeVLQ(buf[pos:msgLen-3], 0) // cycle_ticks
			pos += np
			value, np := decodeVLQ(buf[pos:msgLen-3], 0)
			pos += np
			defaultVal, _ := decodeVLQ(buf[pos:msgLen-3], 0)
			if trace {
				fmt.Printf("  config_pwm_out oid=%d pin=%d value=%d default=%d\n",
					oid, pin, value, defaultVal)
			}
			state.mu.Lock()
			state.pwmOuts[int(oid)] = &PWMOutState{
				oid:      int(oid),
				pin:      int(pin),
				value:    uint16(value),
				maxValue: 255,
			}
			state.mu.Unlock()

		case CMD_SCHED_PWM_OUT:
			oid, np := decodeVLQ(buf[pos:msgLen-3], 0)
			pos += np
			_, np = decodeVLQ(buf[pos:msgLen-3], 0) // clock
			pos += np
			value, _ := decodeVLQ(buf[pos:msgLen-3], 0)
			if trace {
				fmt.Printf("  schedule_pwm_out oid=%d value=%d\n", oid, value)
			}
			state.mu.Lock()
			if p, ok := state.pwmOuts[int(oid)]; ok {
				p.value = uint16(value)
			}
			state.mu.Unlock()

		case CMD_CONFIG_ANALOG_IN:
			oid, np := decodeVLQ(buf[pos:msgLen-3], 0)
			pos += np
			pin, _ := decodeVLQ(buf[pos:msgLen-3], 0)
			if trace {
				fmt.Printf("  config_analog_in oid=%d pin=%d\n", oid, pin)
			}
			state.mu.Lock()
			state.analogIns[int(oid)] = &AnalogInState{
				oid: int(oid),
				pin: int(pin),
			}
			state.mu.Unlock()

		case CMD_QUERY_ANALOG_IN:
			oid, np := decodeVLQ(buf[pos:msgLen-3], 0)
			pos += np
			clock, np := decodeVLQ(buf[pos:msgLen-3], 0)
			pos += np
			sampleTicks, np := decodeVLQ(buf[pos:msgLen-3], 0)
			pos += np
			sampleCount, np := decodeVLQ(buf[pos:msgLen-3], 0)
			pos += np
			restTicks, _ := decodeVLQ(buf[pos:msgLen-3], 0)
			if trace {
				fmt.Printf("  query_analog_in oid=%d clock=%d sample_ticks=%d count=%d rest=%d\n",
					oid, clock, sampleTicks, sampleCount, restTicks)
			}
			state.mu.Lock()
			if a, ok := state.analogIns[int(oid)]; ok {
				a.sampleTime = uint32(sampleTicks)
				a.sampleCount = int(sampleCount)
				a.reportClock = uint32(clock)
				a.reportTicks = uint32(restTicks)
			}
			state.mu.Unlock()

		case CMD_CONFIG_ENDSTOP:
			oid, np := decodeVLQ(buf[pos:msgLen-3], 0)
			pos += np
			pin, np := decodeVLQ(buf[pos:msgLen-3], 0)
			pos += np
			pullup, _ := decodeVLQ(buf[pos:msgLen-3], 0)
			if trace {
				fmt.Printf("  config_endstop oid=%d pin=%d pullup=%d\n", oid, pin, pullup)
			}
			state.mu.Lock()
			state.endstops[int(oid)] = &EndstopState{
				oid:    int(oid),
				pin:    int(pin),
				pullup: pullup != 0,
			}
			state.mu.Unlock()

		case CMD_ENDSTOP_QUERY:
			oid, _ := decodeVLQ(buf[pos:msgLen-3], 0)
			state.mu.Lock()
			triggered := false
			if e, ok := state.endstops[int(oid)]; ok {
				triggered = e.triggered
			}
			state.mu.Unlock()
			pinVal := byte(0)
			if triggered {
				pinVal = 1
			}
			if trace {
				fmt.Printf("  endstop_query oid=%d -> pin=%d\n", oid, pinVal)
			}
			encodeVLQ(&resp, RESP_ENDSTOP_STATE)
			encodeVLQ(&resp, oid)
			encodeVLQ(&resp, 0) // homing
			encodeVLQ(&resp, int32(state.GetClock()))
			encodeVLQ(&resp, int32(pinVal))

		case CMD_ENDSTOP_HOME:
			oid, np := decodeVLQ(buf[pos:msgLen-3], 0)
			pos += np
			clock, _ := decodeVLQ(buf[pos:msgLen-3], 0)
			if trace {
				fmt.Printf("  endstop_home oid=%d clock=%d\n", oid, clock)
			}
			// Simulate endstop trigger after a delay
			go func(oid int32) {
				time.Sleep(500 * time.Millisecond)
				state.mu.Lock()
				if e, ok := state.endstops[int(oid)]; ok {
					e.triggered = true
				}
				state.mu.Unlock()
			}(oid)

		default:
			if trace {
				fmt.Printf("  Unknown command ID: %d\n", cmdID)
			}
		}

		// Send response
		if len(resp) > 0 {
			msg := buildMessage(seq|0x10, resp)
			conn.Write(msg)
		} else {
			// ACK
			ack := buildMessage(seq|0x10, nil)
			conn.Write(ack)
		}
		seq = (seq + 1) & 0x0f
	}
}
