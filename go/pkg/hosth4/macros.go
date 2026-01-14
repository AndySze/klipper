package hosth4

import (
	"bufio"
	"fmt"
	"os"
	"strconv"
	"strings"
)

type macroEngine struct {
	macros map[string]*macroDef
	vars   map[string]map[string]float64
}

type macroDef struct {
	name string
	body []string
	vars map[string]float64
}

func loadGCodeMacros(cfgPath string) (*macroEngine, error) {
	f, err := os.Open(cfgPath)
	if err != nil {
		return nil, err
	}
	defer f.Close()

	e := &macroEngine{
		macros: map[string]*macroDef{},
		vars:   map[string]map[string]float64{},
	}

	var cur *macroDef
	inGCode := false

	flushCur := func() {
		if cur == nil || cur.name == "" {
			cur = nil
			inGCode = false
			return
		}
		name := strings.ToUpper(cur.name)
		e.macros[name] = cur
		if _, ok := e.vars[name]; !ok {
			e.vars[name] = map[string]float64{}
		}
		for k, v := range cur.vars {
			e.vars[name][k] = v
		}
		cur = nil
		inGCode = false
	}

	s := bufio.NewScanner(f)
	for s.Scan() {
		rawLine := s.Text()
		line := strings.TrimRight(rawLine, "\r\n")
		trim := strings.TrimSpace(line)
		if trim == "" {
			continue
		}
		// Strip full-line comments for config parsing.
		if strings.HasPrefix(trim, "#") {
			continue
		}

		// Section header (allow trailing comments after ']').
		if strings.HasPrefix(trim, "[") {
			end := strings.IndexByte(trim, ']')
			if end > 0 {
				sec := strings.TrimSpace(trim[1:end])
				if strings.HasPrefix(strings.ToLower(sec), "gcode_macro ") {
					flushCur()
					name := strings.TrimSpace(sec[len("gcode_macro "):])
					cur = &macroDef{name: name, vars: map[string]float64{}}
					continue
				}
			}
			// Any other section ends macro parsing state.
			flushCur()
			continue
		}

		if cur == nil {
			continue
		}

		// In a gcode: block, accept indented lines.
		if inGCode {
			if len(line) > 0 && (line[0] == ' ' || line[0] == '\t') {
				cur.body = append(cur.body, strings.TrimSpace(line))
				continue
			}
			// Non-indented line ends the gcode block (fallthrough to parse as key).
			inGCode = false
		}

		// Parse key/value line (strip trailing '#' comment).
		cfgLine := line
		if idx := strings.IndexByte(cfgLine, '#'); idx >= 0 {
			cfgLine = strings.TrimSpace(cfgLine[:idx])
			if cfgLine == "" {
				continue
			}
		}
		kv := strings.SplitN(cfgLine, ":", 2)
		if len(kv) != 2 {
			kv = strings.SplitN(cfgLine, "=", 2)
		}
		if len(kv) != 2 {
			continue
		}
		key := strings.TrimSpace(kv[0])
		val := strings.TrimSpace(kv[1])
		if key == "" {
			continue
		}

		switch {
		case strings.EqualFold(key, "gcode"):
			if val != "" {
				cur.body = append(cur.body, strings.TrimSpace(val))
			}
			inGCode = true
		case strings.HasPrefix(strings.ToLower(key), "variable_"):
			name := strings.TrimSpace(key[len("variable_"):])
			if name == "" || val == "" {
				continue
			}
			fv, err := strconv.ParseFloat(val, 64)
			if err != nil {
				return nil, fmt.Errorf("bad macro variable %s=%q: %w", key, val, err)
			}
			cur.vars[strings.ToLower(name)] = fv
		default:
			// Ignore other keys (description, etc).
		}
	}
	if err := s.Err(); err != nil {
		return nil, err
	}
	flushCur()
	if len(e.macros) == 0 {
		return nil, nil
	}
	return e, nil
}

func (e *macroEngine) has(name string) bool {
	if e == nil {
		return false
	}
	_, ok := e.macros[strings.ToUpper(name)]
	return ok
}

func (e *macroEngine) setVariable(args map[string]string) error {
	if e == nil {
		return nil
	}
	macro := strings.ToUpper(strings.TrimSpace(args["MACRO"]))
	varName := strings.ToLower(strings.TrimSpace(args["VARIABLE"]))
	valRaw := strings.TrimSpace(args["VALUE"])
	if macro == "" || varName == "" {
		return fmt.Errorf("SET_GCODE_VARIABLE missing MACRO/VARIABLE")
	}
	if valRaw == "" {
		return fmt.Errorf("SET_GCODE_VARIABLE missing VALUE")
	}
	v, err := strconv.ParseFloat(valRaw, 64)
	if err != nil {
		return fmt.Errorf("SET_GCODE_VARIABLE bad VALUE=%q", valRaw)
	}
	if _, ok := e.vars[macro]; !ok {
		e.vars[macro] = map[string]float64{}
	}
	e.vars[macro][varName] = v
	return nil
}

func (e *macroEngine) expand(rt *runtime, name string, params map[string]string) ([]string, error) {
	if e == nil {
		return nil, fmt.Errorf("macro engine not initialized")
	}
	m := e.macros[strings.ToUpper(name)]
	if m == nil {
		return nil, fmt.Errorf("unknown macro %q", name)
	}
	ctx := &macroEvalContext{
		rt:      rt,
		engine:  e,
		macro:   strings.ToUpper(name),
		params:  params,
		printer: map[string]bool{"toolhead": true, "gcode_move": true},
	}
	return renderMacro(m.body, ctx)
}

type macroEvalContext struct {
	rt      *runtime
	engine  *macroEngine
	macro   string
	params  map[string]string
	printer map[string]bool
}

func (c *macroEvalContext) floatVar(name string) (float64, bool) {
	if c.engine == nil {
		return 0.0, false
	}
	vars := c.engine.vars[c.macro]
	if vars == nil {
		return 0.0, false
	}
	v, ok := vars[strings.ToLower(name)]
	return v, ok
}

func (c *macroEvalContext) floatParam(name string) (float64, bool) {
	raw := strings.TrimSpace(c.params[strings.ToUpper(name)])
	if raw == "" {
		return 0.0, false
	}
	v, err := strconv.ParseFloat(raw, 64)
	if err != nil {
		return 0.0, false
	}
	return v, true
}

func (c *macroEvalContext) strParam(name string) (string, bool) {
	raw, ok := c.params[strings.ToUpper(name)]
	if !ok {
		return "", false
	}
	return raw, true
}

func (c *macroEvalContext) positionAxis(axis byte) float64 {
	if c.rt == nil || c.rt.toolhead == nil {
		return 0.0
	}
	if len(c.rt.toolhead.commandedPos) < 3 {
		return 0.0
	}
	switch axis {
	case 'X':
		return c.rt.toolhead.commandedPos[0]
	case 'Y':
		return c.rt.toolhead.commandedPos[1]
	case 'Z':
		return c.rt.toolhead.commandedPos[2]
	default:
		return 0.0
	}
}

func renderMacro(lines []string, ctx *macroEvalContext) ([]string, error) {
	type frame struct {
		parentActive bool
		cond         bool
		inElse       bool
	}
	active := true
	var stack []frame
	var out []string

	for _, raw := range lines {
		line := strings.TrimSpace(raw)
		if line == "" {
			continue
		}
		if strings.HasPrefix(line, "{") && strings.Contains(line, "action_respond_info") {
			continue
		}
		if strings.HasPrefix(line, "{%") && strings.HasSuffix(line, "%}") {
			inner := strings.TrimSpace(strings.TrimSuffix(strings.TrimPrefix(line, "{%"), "%}"))
			switch {
			case strings.HasPrefix(inner, "if "):
				parent := active
				cond := false
				if parent {
					v, err := evalBoolExpr(strings.TrimSpace(inner[len("if "):]), ctx)
					if err != nil {
						return nil, err
					}
					cond = v
				}
				stack = append(stack, frame{parentActive: parent, cond: cond})
				active = parent && cond
			case inner == "else":
				if len(stack) == 0 {
					return nil, fmt.Errorf("macro else without if")
				}
				f := stack[len(stack)-1]
				f.inElse = true
				stack[len(stack)-1] = f
				active = f.parentActive && !f.cond
			case inner == "endif":
				if len(stack) == 0 {
					return nil, fmt.Errorf("macro endif without if")
				}
				stack = stack[:len(stack)-1]
				active = true
				for _, f := range stack {
					if f.inElse {
						active = active && f.parentActive && !f.cond
					} else {
						active = active && f.parentActive && f.cond
					}
				}
			default:
				// Unknown control tag; ignore.
			}
			continue
		}
		if !active {
			continue
		}
		// Ignore standalone { ... } action blocks.
		if strings.HasPrefix(line, "{") && strings.HasSuffix(line, "}") && !strings.Contains(line, "{% ") {
			continue
		}
		// Substitute inline {expr} expressions
		rendered, err := substituteInlineExprs(line, ctx)
		if err != nil {
			return nil, err
		}
		out = append(out, rendered)
	}
	return out, nil
}

// substituteInlineExprs replaces {expr} expressions in a line with their values.
func substituteInlineExprs(line string, ctx *macroEvalContext) (string, error) {
	result := line
	for {
		start := strings.Index(result, "{")
		if start == -1 {
			break
		}
		// Skip {% ... %} control blocks
		if start+1 < len(result) && result[start+1] == '%' {
			break
		}
		end := strings.Index(result[start:], "}")
		if end == -1 {
			break
		}
		end += start
		expr := result[start+1 : end]
		value, err := evalInlineExpr(expr, ctx)
		if err != nil {
			// If we can't evaluate, skip this expression
			break
		}
		result = result[:start] + value + result[end+1:]
	}
	return result, nil
}

// evalInlineExpr evaluates an inline expression like "params.L|int"
func evalInlineExpr(expr string, ctx *macroEvalContext) (string, error) {
	expr = strings.TrimSpace(expr)

	// Handle filter: expr|filter
	if strings.Contains(expr, "|") {
		parts := strings.SplitN(expr, "|", 2)
		baseExpr := strings.TrimSpace(parts[0])
		filter := strings.TrimSpace(parts[1])

		// Evaluate base expression
		value, err := evalInlineExpr(baseExpr, ctx)
		if err != nil {
			return "", err
		}

		// Apply filter
		switch filter {
		case "int":
			// Parse as float and convert to int
			f, err := strconv.ParseFloat(value, 64)
			if err != nil {
				return "", fmt.Errorf("cannot convert %q to int: %w", value, err)
			}
			return strconv.Itoa(int(f)), nil
		case "float":
			return value, nil
		default:
			return "", fmt.Errorf("unknown filter: %q", filter)
		}
	}

	// Handle params.X
	if strings.HasPrefix(expr, "params.") {
		paramName := strings.TrimPrefix(expr, "params.")
		value, ok := ctx.params[strings.ToUpper(paramName)]
		if !ok {
			return "", fmt.Errorf("undefined parameter: %q", paramName)
		}
		return value, nil
	}

	// Handle printer.toolhead.extruder - returns the name of the currently active extruder
	if expr == "printer.toolhead.extruder" {
		if ctx.rt != nil && ctx.rt.activeExtruder != "" {
			return ctx.rt.activeExtruder, nil
		}
		return "extruder", nil
	}

	// Return as-is for now
	return expr, nil
}

func evalBoolExpr(expr string, ctx *macroEvalContext) (bool, error) {
	// Support a tiny subset of Jinja expressions used by macros.cfg.
	expr = strings.TrimSpace(expr)
	if expr == "" {
		return false, fmt.Errorf("empty expression")
	}
	parts := splitTopLevel(expr, " or ")
	if len(parts) > 1 {
		for _, p := range parts {
			v, err := evalBoolExpr(p, ctx)
			if err != nil {
				return false, err
			}
			if v {
				return true, nil
			}
		}
		return false, nil
	}
	parts = splitTopLevel(expr, " and ")
	if len(parts) > 1 {
		for _, p := range parts {
			v, err := evalBoolExpr(p, ctx)
			if err != nil {
				return false, err
			}
			if !v {
				return false, nil
			}
		}
		return true, nil
	}

	if strings.Contains(expr, " not in ") {
		kv := strings.SplitN(expr, " not in ", 2)
		left := strings.TrimSpace(kv[0])
		right := strings.TrimSpace(kv[1])
		if right != "printer" {
			return false, fmt.Errorf("unsupported not in rhs: %q", right)
		}
		s, err := evalString(left, ctx)
		if err != nil {
			return false, err
		}
		return !ctx.printer[strings.ToLower(s)], nil
	}
	if strings.Contains(expr, " in ") {
		kv := strings.SplitN(expr, " in ", 2)
		left := strings.TrimSpace(kv[0])
		right := strings.TrimSpace(kv[1])
		if right != "printer" {
			return false, fmt.Errorf("unsupported in rhs: %q", right)
		}
		s, err := evalString(left, ctx)
		if err != nil {
			return false, err
		}
		return ctx.printer[strings.ToLower(s)], nil
	}

	// Handle "params.X is not defined"
	if strings.Contains(expr, " is not defined") {
		varExpr := strings.TrimSuffix(expr, " is not defined")
		varExpr = strings.TrimSpace(varExpr)
		if strings.HasPrefix(varExpr, "params.") {
			paramName := strings.TrimPrefix(varExpr, "params.")
			_, exists := ctx.params[strings.ToUpper(paramName)]
			return !exists, nil
		}
		return false, fmt.Errorf("unsupported 'is not defined' expression: %q", varExpr)
	}
	// Handle "params.X is defined"
	if strings.Contains(expr, " is defined") {
		varExpr := strings.TrimSuffix(expr, " is defined")
		varExpr = strings.TrimSpace(varExpr)
		if strings.HasPrefix(varExpr, "params.") {
			paramName := strings.TrimPrefix(varExpr, "params.")
			_, exists := ctx.params[strings.ToUpper(paramName)]
			return exists, nil
		}
		return false, fmt.Errorf("unsupported 'is defined' expression: %q", varExpr)
	}

	if strings.Contains(expr, "!=") {
		kv := strings.SplitN(expr, "!=", 2)
		return evalCompare(strings.TrimSpace(kv[0]), strings.TrimSpace(kv[1]), false, ctx)
	}
	if strings.Contains(expr, "==") {
		kv := strings.SplitN(expr, "==", 2)
		return evalCompare(strings.TrimSpace(kv[0]), strings.TrimSpace(kv[1]), true, ctx)
	}
	return false, fmt.Errorf("unsupported expression: %q", expr)
}

func evalCompare(a, b string, eq bool, ctx *macroEvalContext) (bool, error) {
	af, aok := evalFloat(a, ctx)
	bf, bok := evalFloat(b, ctx)
	if aok && bok {
		if eq {
			return af == bf, nil
		}
		return af != bf, nil
	}
	as, err := evalString(a, ctx)
	if err != nil {
		return false, err
	}
	bs, err := evalString(b, ctx)
	if err != nil {
		return false, err
	}
	if eq {
		return as == bs, nil
	}
	return as != bs, nil
}

func evalString(expr string, ctx *macroEvalContext) (string, error) {
	expr = strings.TrimSpace(expr)
	if strings.HasPrefix(expr, "\"") && strings.HasSuffix(expr, "\"") && len(expr) >= 2 {
		return expr[1 : len(expr)-1], nil
	}
	// params.T
	if strings.HasPrefix(expr, "params.") {
		key := strings.TrimSpace(expr[len("params."):])
		v, ok := ctx.strParam(key)
		if !ok {
			return "", nil
		}
		return v, nil
	}
	return expr, nil
}

func evalFloat(expr string, ctx *macroEvalContext) (float64, bool) {
	expr = strings.TrimSpace(expr)
	if expr == "" {
		return 0.0, false
	}
	if strings.Contains(expr, "-") {
		parts := splitTopLevel(expr, " - ")
		if len(parts) == 2 {
			a, okA := evalFloat(parts[0], ctx)
			b, okB := evalFloat(parts[1], ctx)
			if okA && okB {
				return a - b, true
			}
			return 0.0, false
		}
	}
	if v, err := strconv.ParseFloat(expr, 64); err == nil {
		return v, true
	}
	if strings.HasPrefix(expr, "printer.toolhead.position.") {
		a := strings.TrimSpace(expr[len("printer.toolhead.position."):])
		if len(a) >= 1 {
			return ctx.positionAxis(byte(strings.ToUpper(a[:1])[0])), true
		}
	}
	if strings.HasPrefix(expr, "printer.gcode_move.gcode_position.") {
		a := strings.TrimSpace(expr[len("printer.gcode_move.gcode_position."):])
		if len(a) >= 1 {
			return ctx.positionAxis(byte(strings.ToUpper(a[:1])[0])), true
		}
	}
	// printer["gcode_macro NAME"].t
	if strings.HasPrefix(expr, "printer[\"gcode_macro ") && strings.Contains(expr, "\"].") {
		end := strings.Index(expr, "\"].")
		name := expr[len("printer[\"gcode_macro "):end]
		rest := expr[end+len("\"]."):]
		varName := strings.ToLower(strings.TrimSpace(rest))
		macro := strings.ToUpper(strings.TrimSpace(name))
		if ctx.engine != nil {
			if v, ok := ctx.engine.vars[macro][varName]; ok {
				return v, true
			}
		}
		return 0.0, false
	}
	// local variable (e.g., "t")
	if v, ok := ctx.floatVar(expr); ok {
		return v, true
	}
	// params (numeric)
	if strings.HasPrefix(expr, "params.") {
		key := strings.TrimSpace(expr[len("params."):])
		return ctx.floatParam(key)
	}
	return 0.0, false
}

func splitTopLevel(s string, sep string) []string {
	if !strings.Contains(s, sep) {
		return []string{s}
	}
	// No need for full quoting/paren awareness for macros.cfg; keep it simple.
	return strings.Split(s, sep)
}

func execGCodeWithMacros(rt *runtime, macros *macroEngine, line string, depth int) error {
	if depth > 32 {
		return fmt.Errorf("macro recursion too deep")
	}
	cmd, err := parseGCodeLine(line)
	if err != nil {
		return err
	}
	if cmd == nil {
		return nil
	}
	if macros != nil {
		switch cmd.Name {
		case "SET_GCODE_VARIABLE":
			return macros.setVariable(cmd.Args)
		}
		if macros.has(cmd.Name) {
			expanded, err := macros.expand(rt, cmd.Name, cmd.Args)
			if err != nil {
				return err
			}
			for _, ln := range expanded {
				if err := execGCodeWithMacros(rt, macros, ln, depth+1); err != nil {
					return err
				}
			}
			return nil
		}
	}
	return rt.exec(cmd)
}

// HomingOverride represents parsed [homing_override] configuration
type HomingOverride struct {
	Gcode        []string
	SetPositionX *float64
	SetPositionY *float64
	SetPositionZ *float64
	Axes         string // axes to mark as homed (e.g., "xyz")
}

// loadHomingOverride parses the [homing_override] section from a config file.
// It handles multi-line gcode blocks properly (indented continuation lines).
func loadHomingOverride(cfgPath string) (*HomingOverride, error) {
	f, err := os.Open(cfgPath)
	if err != nil {
		return nil, err
	}
	defer f.Close()

	var ho *HomingOverride
	inSection := false
	inGCode := false

	s := bufio.NewScanner(f)
	for s.Scan() {
		rawLine := s.Text()
		line := strings.TrimRight(rawLine, "\r\n")
		trim := strings.TrimSpace(line)
		if trim == "" {
			continue
		}
		// Strip full-line comments
		if strings.HasPrefix(trim, "#") {
			continue
		}

		// Section header
		if strings.HasPrefix(trim, "[") {
			end := strings.IndexByte(trim, ']')
			if end > 0 {
				sec := strings.TrimSpace(trim[1:end])
				if strings.EqualFold(sec, "homing_override") {
					inSection = true
					ho = &HomingOverride{}
					continue
				}
			}
			// Any other section ends homing_override parsing
			if inSection {
				break
			}
			continue
		}

		if !inSection {
			continue
		}

		// In a gcode: block, accept indented lines
		if inGCode {
			if len(line) > 0 && (line[0] == ' ' || line[0] == '\t') {
				ho.Gcode = append(ho.Gcode, strings.TrimSpace(line))
				continue
			}
			// Non-indented line ends the gcode block
			inGCode = false
		}

		// Parse key/value line (strip trailing '#' comment)
		cfgLine := line
		if idx := strings.IndexByte(cfgLine, '#'); idx >= 0 {
			cfgLine = strings.TrimSpace(cfgLine[:idx])
			if cfgLine == "" {
				continue
			}
		}
		kv := strings.SplitN(cfgLine, ":", 2)
		if len(kv) != 2 {
			kv = strings.SplitN(cfgLine, "=", 2)
		}
		if len(kv) != 2 {
			continue
		}
		key := strings.TrimSpace(kv[0])
		val := strings.TrimSpace(kv[1])
		if key == "" {
			continue
		}

		switch strings.ToLower(key) {
		case "gcode":
			if val != "" {
				ho.Gcode = append(ho.Gcode, strings.TrimSpace(val))
			}
			inGCode = true
		case "set_position_x":
			if val != "" {
				v, err := strconv.ParseFloat(val, 64)
				if err == nil {
					ho.SetPositionX = &v
				}
			}
		case "set_position_y":
			if val != "" {
				v, err := strconv.ParseFloat(val, 64)
				if err == nil {
					ho.SetPositionY = &v
				}
			}
		case "set_position_z":
			if val != "" {
				v, err := strconv.ParseFloat(val, 64)
				if err == nil {
					ho.SetPositionZ = &v
				}
			}
		case "axes":
			ho.Axes = strings.ToLower(val)
		}
	}
	if err := s.Err(); err != nil {
		return nil, err
	}
	return ho, nil
}
