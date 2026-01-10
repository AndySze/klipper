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
		// Ignore other { ... } action blocks.
		if strings.HasPrefix(line, "{") && strings.HasSuffix(line, "}") {
			continue
		}
		out = append(out, line)
	}
	return out, nil
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
