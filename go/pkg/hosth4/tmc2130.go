package hosth4

import (
	"fmt"
	"os"
)

// TMC2130 register addresses (matching hosth1)
const (
	tmc2130RegGCONF      = 0x00
	tmc2130RegGSTAT      = 0x01
	tmc2130RegIHOLD_IRUN = 0x10
	tmc2130RegTPOWERDOWN = 0x11
	tmc2130RegTPWMTHRS   = 0x13
	tmc2130RegTCOOLTHRS  = 0x14
	tmc2130RegTHIGH      = 0x15
	tmc2130RegMSLUT0     = 0x60
	tmc2130RegMSLUTSEL   = 0x68
	tmc2130RegMSLUTSTART = 0x69
	tmc2130RegMSCNT      = 0x6a
	tmc2130RegCHOPCONF   = 0x6c
	tmc2130RegCOOLCONF   = 0x6d
	tmc2130RegDRV_STATUS = 0x6f
	tmc2130RegPWMCONF    = 0x70
)

// tmc2130DefaultMSLUT is the default wave table for TMC2130
var tmc2130DefaultMSLUT = [8]uint32{
	0x54aab5aa, 0xaa54954a, 0x29244929, 0x22421010,
	0xfffffffb, 0x7d77bbb5, 0x56552949, 0x22420040,
}

// tmc2130Driver manages a single TMC2130 stepper driver
type tmc2130Driver struct {
	rt *runtime

	// SPI communication
	spiOID int

	// Driver configuration
	runCurrent    float64
	senseResistor float64
	microsteps    int

	// Calculated register values (cached from init)
	chopconf    uint32
	iholdIrun   uint32
	mslutsel    uint32
	mslutstart  uint32
	pwmconf     uint32

	// State tracking
	initSentDuringRun bool
}

// newTMC2130Driver creates a new TMC2130 driver instance
func newTMC2130Driver(rt *runtime, spiOID int, runCurrent, senseResistor float64, microsteps int) *tmc2130Driver {
	d := &tmc2130Driver{
		rt:            rt,
		spiOID:        spiOID,
		runCurrent:    runCurrent,
		senseResistor: senseResistor,
		microsteps:    microsteps,
	}
	d.calculateRegisters()
	return d
}

// calculateRegisters computes the register values based on configuration
func (d *tmc2130Driver) calculateRegisters() {
	// Calculate current
	holdCurrent := 2.0 // MAX_CURRENT default for hold
	vsense, irun, ihold := tmc2130CalcCurrent(d.runCurrent, holdCurrent, d.senseResistor)

	// Build register values
	mres := microstepsToMres(d.microsteps)
	vsenseInt := 0
	if vsense {
		vsenseInt = 1
	}

	d.chopconf = tmc2130BuildCHOPCONF(4, 0, 7, 1, vsenseInt, mres, true) // toff=4, hstrt=0, hend=7, tbl=1, intpol=true
	d.iholdIrun = tmc2130BuildIHOLD_IRUN(ihold, irun, 8)                 // iholddelay=8
	d.mslutsel = tmc2130BuildMSLUTSEL(2, 1, 1, 1, 128, 255, 255)         // w0=2, w1=1, w2=1, w3=1, x1=128, x2=255, x3=255
	d.mslutstart = tmc2130BuildMSLUTSTART(0, 247)                        // start_sin=0, start_sin90=247
	d.pwmconf = tmc2130BuildPWMCONF(128, 4, 1, true, 0)                  // pwm_ampl=128, pwm_grad=4, pwm_freq=1, pwm_autoscale=true
}

// initRegisters sends all TMC2130 initialization commands
// This matches Python's TMCCommandHelper._init_registers()
func (d *tmc2130Driver) initRegisters() error {
	if d.rt == nil {
		fmt.Fprintf(os.Stderr, "TMC: initRegisters ERROR: rt is nil\n")
		return fmt.Errorf("tmc2130: runtime not initialized")
	}
	if d.rt.cqMain == nil {
		fmt.Fprintf(os.Stderr, "TMC: initRegisters ERROR: cqMain is nil\n")
		return fmt.Errorf("tmc2130: cqMain not initialized")
	}
	if d.rt.sq == nil {
		fmt.Fprintf(os.Stderr, "TMC: initRegisters ERROR: sq is nil\n")
		return fmt.Errorf("tmc2130: sq not initialized")
	}
	fmt.Fprintf(os.Stderr, "TMC: initRegisters spiOID=%d, chopconf=0x%08x, sq=%p, cqMain=%p\n", d.spiOID, d.chopconf, d.rt.sq, d.rt.cqMain)

	// Order matches Python: CHOPCONF, IHOLD_IRUN, MSLUT0-7, MSLUTSEL, MSLUTSTART,
	// TPWMTHRS, GCONF, TCOOLTHRS, THIGH, COOLCONF, PWMCONF, TPOWERDOWN
	if err := d.writeRegister(tmc2130RegCHOPCONF, d.chopconf); err != nil {
		return err
	}
	if err := d.writeRegister(tmc2130RegIHOLD_IRUN, d.iholdIrun); err != nil {
		return err
	}

	// MSLUT0-7
	for i := 0; i < 8; i++ {
		if err := d.writeRegister(byte(tmc2130RegMSLUT0+i), tmc2130DefaultMSLUT[i]); err != nil {
			return err
		}
	}

	if err := d.writeRegister(tmc2130RegMSLUTSEL, d.mslutsel); err != nil {
		return err
	}
	if err := d.writeRegister(tmc2130RegMSLUTSTART, d.mslutstart); err != nil {
		return err
	}
	if err := d.writeRegister(tmc2130RegTPWMTHRS, 0x000FFFFF); err != nil {
		return err
	}
	if err := d.writeRegister(tmc2130RegGCONF, 0); err != nil {
		return err
	}
	if err := d.writeRegister(tmc2130RegTCOOLTHRS, 0); err != nil {
		return err
	}
	if err := d.writeRegister(tmc2130RegTHIGH, 0); err != nil {
		return err
	}
	if err := d.writeRegister(tmc2130RegCOOLCONF, 0); err != nil {
		return err
	}
	if err := d.writeRegister(tmc2130RegPWMCONF, d.pwmconf); err != nil {
		return err
	}
	if err := d.writeRegister(tmc2130RegTPOWERDOWN, 0); err != nil {
		return err
	}

	return nil
}

// readPhaseOffset reads DRV_STATUS, GSTAT, and MSCNT for phase offset calculation
// This matches Python's TMCMicrostepHelper.get_phase_offset()
func (d *tmc2130Driver) readPhaseOffset() error {
	// Read DRV_STATUS (0x6f = 'o')
	if err := d.readRegister(tmc2130RegDRV_STATUS); err != nil {
		return err
	}
	// Read GSTAT (0x01)
	if err := d.readRegister(tmc2130RegGSTAT); err != nil {
		return err
	}
	// Read MSCNT (0x6a = 'j')
	if err := d.readRegister(tmc2130RegMSCNT); err != nil {
		return err
	}
	return nil
}

// writeRegister sends an SPI write command to the TMC2130
func (d *tmc2130Driver) writeRegister(reg byte, value uint32) error {
	// TMC2130 SPI format: 5 bytes
	// Byte 0: register address | 0x80 (write bit)
	// Bytes 1-4: 32-bit value (big-endian)
	data := []byte{
		reg | 0x80,
		byte(value >> 24),
		byte(value >> 16),
		byte(value >> 8),
		byte(value),
	}
	cmd := fmt.Sprintf("spi_send oid=%d data=%s", d.spiOID, formatBytes(data))
	fmt.Fprintf(os.Stderr, "TMC: writeRegister reg=0x%02x cmd=%s\n", reg, cmd)
	err := d.rt.sendLine(cmd, d.rt.cqMain, 0, 0)
	if err != nil {
		fmt.Fprintf(os.Stderr, "TMC: writeRegister ERROR: %v\n", err)
	}
	return err
}

// readRegister sends an SPI read command to the TMC2130
func (d *tmc2130Driver) readRegister(reg byte) error {
	// TMC2130 SPI read format: 5 bytes
	// Byte 0: register address (no write bit)
	// Bytes 1-4: dummy bytes (will be returned with previous read result)
	data := []byte{reg, 0, 0, 0, 0}
	cmd := fmt.Sprintf("spi_send oid=%d data=%s", d.spiOID, formatBytes(data))
	return d.rt.sendLine(cmd, d.rt.cqMain, 0, 0)
}

// doEnable is called when the associated stepper motor is enabled
// This matches Python's TMCCommandHelper._do_enable()
func (d *tmc2130Driver) doEnable() error {
	fmt.Fprintf(os.Stderr, "TMC: doEnable spiOID=%d, initSentDuringRun=%v\n", d.spiOID, d.initSentDuringRun)
	if d.initSentDuringRun {
		return nil
	}

	// Send init commands
	if err := d.initRegisters(); err != nil {
		fmt.Fprintf(os.Stderr, "TMC: doEnable initRegisters ERROR: %v\n", err)
		return err
	}

	// Read phase offset registers
	if err := d.readPhaseOffset(); err != nil {
		return err
	}

	// Ensure commands are written to output file before continuing
	if d.rt.sq != nil {
		_ = d.rt.sq.WaitDrain()
	}

	d.initSentDuringRun = true
	return nil
}

// Helper functions (matching hosth1/h1.go)

func tmc2130CalcCurrent(runCurrent, holdCurrent, senseResistor float64) (vsense bool, irun, ihold int) {
	irun = tmc2130CalcCurrentBits(runCurrent, senseResistor, true)
	vsense = true
	cur := tmc2130CalcCurrentFromBits(irun, senseResistor, true)
	if runCurrent-cur > 0.0001 {
		irun2 := tmc2130CalcCurrentBits(runCurrent, senseResistor, false)
		cur2 := tmc2130CalcCurrentFromBits(irun2, senseResistor, false)
		if cur2-runCurrent < runCurrent-cur {
			vsense = false
			irun = irun2
		}
	}
	ihold = tmc2130CalcCurrentBits(holdCurrent, senseResistor, vsense)
	if ihold > irun {
		ihold = irun
	}
	return
}

func tmc2130CalcCurrentBits(current, senseResistor float64, vsense bool) int {
	vref := 0.325
	if !vsense {
		vref = 0.180
	}
	cs := int(32.0*current*senseResistor*1.41421/vref - 1.0 + 0.5)
	if cs < 0 {
		cs = 0
	}
	if cs > 31 {
		cs = 31
	}
	return cs
}

func tmc2130CalcCurrentFromBits(cs int, senseResistor float64, vsense bool) float64 {
	vref := 0.325
	if !vsense {
		vref = 0.180
	}
	return float64(cs+1) * vref / (32.0 * senseResistor * 1.41421)
}

func microstepsToMres(microsteps int) int {
	switch microsteps {
	case 256:
		return 0
	case 128:
		return 1
	case 64:
		return 2
	case 32:
		return 3
	case 16:
		return 4
	case 8:
		return 5
	case 4:
		return 6
	case 2:
		return 7
	case 1:
		return 8
	default:
		return 4 // default to 16 microsteps
	}
}

func tmc2130BuildCHOPCONF(toff, hstrt, hend, tbl, vsense, mres int, intpol bool) uint32 {
	v := uint32(toff & 0xf)
	v |= uint32((hstrt&0x7)<<4 | ((hend+3)&0xf)<<7)
	v |= uint32((tbl & 0x3) << 15)
	v |= uint32((vsense & 0x1) << 17)
	v |= uint32((mres & 0xf) << 24)
	if intpol {
		v |= 1 << 28
	}
	return v
}

func tmc2130BuildIHOLD_IRUN(ihold, irun, iholddelay int) uint32 {
	return uint32((ihold & 0x1f) | ((irun & 0x1f) << 8) | ((iholddelay & 0xf) << 16))
}

func tmc2130BuildMSLUTSEL(w0, w1, w2, w3, x1, x2, x3 int) uint32 {
	return uint32((w0&0x3) | ((w1&0x3)<<2) | ((w2&0x3)<<4) | ((w3&0x3)<<6) |
		((x1&0xff)<<8) | ((x2&0xff)<<16) | ((x3&0xff)<<24))
}

func tmc2130BuildMSLUTSTART(start_sin, start_sin90 int) uint32 {
	return uint32((start_sin & 0xff) | ((start_sin90 & 0xff) << 16))
}

func tmc2130BuildPWMCONF(pwm_ampl, pwm_grad, pwm_freq int, pwm_autoscale bool, freewheel int) uint32 {
	v := uint32((pwm_ampl & 0xff) | ((pwm_grad & 0xff) << 8) | ((pwm_freq & 0x3) << 16))
	if pwm_autoscale {
		v |= 1 << 18
	}
	v |= uint32((freewheel & 0x3) << 20)
	return v
}
