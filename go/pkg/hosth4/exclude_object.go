// exclude_object implements object exclusion during prints.
// This allows skipping parts of a print that have failed or are unwanted.
package hosth4

import (
	"encoding/json"
	"fmt"
	"strings"
)

// excludeObject represents an object in the print.
type excludeObject struct {
	name    string
	center  []float64
	polygon [][]float64
	params  map[string]string
}

// excludeObjectManager manages object exclusion during prints.
type excludeObjectManager struct {
	rt               *runtime
	objects          []*excludeObject
	excludedObjects  []string
	currentObject    string
	inExcludedRegion bool
	lastPosition     []float64
}

// newExcludeObjectManager creates a new exclude object manager.
func newExcludeObjectManager(rt *runtime) *excludeObjectManager {
	return &excludeObjectManager{
		rt:               rt,
		objects:          make([]*excludeObject, 0),
		excludedObjects:  make([]string, 0),
		currentObject:    "",
		inExcludedRegion: false,
		lastPosition:     []float64{0, 0, 0, 0},
	}
}

// reset clears all object state.
func (eom *excludeObjectManager) reset() {
	eom.objects = make([]*excludeObject, 0)
	eom.excludedObjects = make([]string, 0)
	eom.currentObject = ""
	eom.inExcludedRegion = false
}

// defineObject adds a new object definition.
func (eom *excludeObjectManager) defineObject(name string, center []float64, polygon [][]float64, params map[string]string) {
	// Check if object already exists
	for _, obj := range eom.objects {
		if obj.name == name {
			return // Already defined
		}
	}

	obj := &excludeObject{
		name:    name,
		center:  center,
		polygon: polygon,
		params:  params,
	}
	eom.objects = append(eom.objects, obj)

	// Sort objects by name
	for i := len(eom.objects) - 1; i > 0; i-- {
		if eom.objects[i].name < eom.objects[i-1].name {
			eom.objects[i], eom.objects[i-1] = eom.objects[i-1], eom.objects[i]
		}
	}
}

// startObject marks the start of an object.
func (eom *excludeObjectManager) startObject(name string) {
	name = strings.ToUpper(name)

	// Auto-define if not yet defined
	found := false
	for _, obj := range eom.objects {
		if obj.name == name {
			found = true
			break
		}
	}
	if !found {
		eom.defineObject(name, nil, nil, nil)
	}

	eom.currentObject = name
}

// endObject marks the end of an object.
func (eom *excludeObjectManager) endObject(name string) error {
	if name != "" && strings.ToUpper(name) != eom.currentObject {
		eom.rt.gcodeRespond(fmt.Sprintf("EXCLUDE_OBJECT_END NAME=%s does not match current object NAME=%s",
			strings.ToUpper(name), eom.currentObject))
	}
	eom.currentObject = ""
	return nil
}

// excludeObject marks an object to be excluded.
func (eom *excludeObjectManager) excludeObject(name string) {
	name = strings.ToUpper(name)
	for _, excluded := range eom.excludedObjects {
		if excluded == name {
			return // Already excluded
		}
	}
	eom.excludedObjects = append(eom.excludedObjects, name)
	eom.rt.gcodeRespond(fmt.Sprintf("Excluding object %s", name))

	// Sort excluded objects by name
	for i := len(eom.excludedObjects) - 1; i > 0; i-- {
		if eom.excludedObjects[i] < eom.excludedObjects[i-1] {
			eom.excludedObjects[i], eom.excludedObjects[i-1] = eom.excludedObjects[i-1], eom.excludedObjects[i]
		}
	}
}

// unexcludeObject removes an object from the exclusion list.
func (eom *excludeObjectManager) unexcludeObject(name string) {
	name = strings.ToUpper(name)
	for i, excluded := range eom.excludedObjects {
		if excluded == name {
			eom.excludedObjects = append(eom.excludedObjects[:i], eom.excludedObjects[i+1:]...)
			eom.rt.gcodeRespond(fmt.Sprintf("Unexcluding object %s", name))
			return
		}
	}
}

// isInExcludedRegion returns true if current moves should be excluded.
func (eom *excludeObjectManager) isInExcludedRegion() bool {
	for _, excluded := range eom.excludedObjects {
		if excluded == eom.currentObject {
			return true
		}
	}
	return false
}

// getStatus returns the current status of exclude_object.
func (eom *excludeObjectManager) getStatus() map[string]any {
	objectList := make([]map[string]any, len(eom.objects))
	for i, obj := range eom.objects {
		objMap := map[string]any{
			"name": obj.name,
		}
		if obj.center != nil {
			objMap["center"] = obj.center
		}
		if obj.polygon != nil {
			objMap["polygon"] = obj.polygon
		}
		for k, v := range obj.params {
			objMap[k] = v
		}
		objectList[i] = objMap
	}

	return map[string]any{
		"objects":          objectList,
		"excluded_objects": eom.excludedObjects,
		"current_object":   eom.currentObject,
	}
}

// cmdExcludeObjectStart handles EXCLUDE_OBJECT_START NAME=<name>
func (eom *excludeObjectManager) cmdExcludeObjectStart(args map[string]string) error {
	name, ok := args["NAME"]
	if !ok || name == "" {
		return fmt.Errorf("EXCLUDE_OBJECT_START requires NAME parameter")
	}
	eom.startObject(name)
	return nil
}

// cmdExcludeObjectEnd handles EXCLUDE_OBJECT_END [NAME=<name>]
func (eom *excludeObjectManager) cmdExcludeObjectEnd(args map[string]string) error {
	name := args["NAME"]
	return eom.endObject(name)
}

// cmdExcludeObject handles EXCLUDE_OBJECT [NAME=<name>] [RESET=1] [CURRENT=1]
func (eom *excludeObjectManager) cmdExcludeObject(args map[string]string) error {
	reset := args["RESET"]
	current := args["CURRENT"]
	name := args["NAME"]

	if reset != "" {
		if name != "" {
			eom.unexcludeObject(name)
		} else {
			eom.excludedObjects = make([]string, 0)
		}
		return nil
	}

	if name != "" {
		eom.excludeObject(name)
		return nil
	}

	if current != "" {
		if eom.currentObject == "" {
			return fmt.Errorf("there is no current object to cancel")
		}
		eom.excludeObject(eom.currentObject)
		return nil
	}

	// List excluded objects
	eom.rt.gcodeRespond(fmt.Sprintf("Excluded objects: %s", strings.Join(eom.excludedObjects, " ")))
	return nil
}

// cmdExcludeObjectDefine handles EXCLUDE_OBJECT_DEFINE [RESET=1] [NAME=<name>] [CENTER=x,y] [POLYGON=[[x,y],...]]
func (eom *excludeObjectManager) cmdExcludeObjectDefine(args map[string]string) error {
	reset := args["RESET"]
	name := args["NAME"]

	if reset != "" {
		eom.reset()
		return nil
	}

	if name == "" {
		// List known objects
		var names []string
		for _, obj := range eom.objects {
			names = append(names, obj.name)
		}
		eom.rt.gcodeRespond(fmt.Sprintf("Known objects: %s", strings.Join(names, " ")))
		return nil
	}

	name = strings.ToUpper(name)

	// Parse center
	var center []float64
	if centerStr, ok := args["CENTER"]; ok {
		if err := json.Unmarshal([]byte("["+centerStr+"]"), &center); err != nil {
			return fmt.Errorf("invalid CENTER: %w", err)
		}
	}

	// Parse polygon
	var polygon [][]float64
	if polygonStr, ok := args["POLYGON"]; ok {
		if err := json.Unmarshal([]byte(polygonStr), &polygon); err != nil {
			return fmt.Errorf("invalid POLYGON: %w", err)
		}
	}

	// Collect other params
	params := make(map[string]string)
	for k, v := range args {
		if k != "NAME" && k != "CENTER" && k != "POLYGON" && k != "RESET" {
			params[k] = v
		}
	}

	eom.defineObject(name, center, polygon, params)
	return nil
}
