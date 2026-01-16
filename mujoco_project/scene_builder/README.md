# MuJoCo Scene Builder

A JSON-to-MJCF pipeline for automatically generating MuJoCo simulation scenes from declarative JSON schemas.

## Overview

This scene builder system allows you to define robotic workcells using simple JSON configuration and automatically generates properly formatted MuJoCo XML (MJCF) files that can be loaded into the MuJoCo physics simulator.

## Architecture

```
schema.json  →  compile_scene.py  →  output_scene.xml  →  mujoco_launcher.py  →  MuJoCo Viewer
(Input)         (Generator)           (MJCF)              (Simulator)            (Visualization)
```

---

## Files

### 1. `compile_scene.py` - Scene Generator

**Purpose:** Converts JSON schema into MuJoCo XML (MJCF) format.

**Key Components:**

#### a. DEFAULTS Library
```python
DEFAULTS = {
    "table": {"size": [0.3, 0.2, 0.025], "rgba": [0.6, 0.6, 0.6, 1]},
    "conveyor": {"size": [0.2, 0.4, 0.025], "rgba": [0.2, 0.2, 0.2, 1]},
    "bin": {"size": [0.15, 0.15, 0.1], "rgba": [0.2, 0.8, 0.2, 0.5]},
    "floor": {"size": [5, 5, 0.1], "rgba": [0.9, 0.9, 0.9, 1]},
    "robot": {"offset": [0, 0, 0]}
}
```
Provides fallback dimensions and colors when not specified in the JSON schema.

#### b. Element Creation
- **`create_element(tag, **attribs)`**: Helper function that creates XML elements with automatic type conversion (lists → space-separated strings).

#### c. Component Processing

**Floor:**
- Creates a plane geom directly in worldbody
- Uses MuJoCo's infinite plane type for ground collision

**Robot:**
- **Critical Pattern**: Robot XML is included at the **root level** (not inside a body)
  ```xml
  <mujoco>
    <!-- ROBOT_FOLDER:franka_emika_panda -->
    <include file="panda.xml"/>
    <worldbody>
      ...
    </worldbody>
  </mujoco>
  ```
- The comment `<!-- ROBOT_FOLDER:... -->` is used by `mujoco_launcher.py` to find the correct robot directory
- Creates a visual pedestal/mount at the specified position
- Maps robot model names to their XML filenames:
  - `franka_emika_panda` → `panda.xml`
  - `franka_fr3` → `fr3.xml`
  - `universal_robots_ur5e` → `ur5e.xml`

**Tables/Conveyors:**
- Creates a body at specified position
- Adds main surface geom (box)
- **Auto-generates legs** using `add_legs()`:
  - 4 cylindrical legs at corners (80% of surface width/length)
  - Legs extend from table center down to floor
  - Position calculation: `leg_z = -table_z / 2` (relative to table center)
- **Conveyor rails**: Adds yellow guide rails along the edges
- **Camera target**: Tables get a target body for camera tracking

**Cameras:**
- Creates a pole from floor to camera height
- Adds a visible red camera box for visualization
- Attaches camera sensor with target tracking
- Target defaults to first table's target body

**Objects:**
- **Critical**: Objects with `freejoint` MUST be direct children of worldbody (MuJoCo requirement)
- Calculates absolute position if parented to a table:
  ```python
  final_pos[2] = parent_pos[2] + parent_thickness + relative_pos[2]
  ```
- Adds freejoint for physics simulation (6-DOF freedom)

#### d. XML Output
- Uses `minidom` for pretty-printing with proper indentation
- Generates `output_scene.xml` in the same directory

**Usage:**
```bash
python compile_scene.py [schema_file]
```
Default: `schema.json` → `output_scene.xml`

---

### 2. `mujoco_launcher.py` - Simulator Launcher

**Purpose:** Loads the generated MJCF and launches MuJoCo viewer with proper path handling for robot includes.

**Critical Implementation Detail:**

MuJoCo robot models in the `mujoco_menagerie` use **relative includes** for assets (meshes, textures). The launcher must change the working directory to the robot's folder before loading.

**The Pattern (from `panda_pick.py`):**
```python
# 1. Read XML as string
with open(scene_path) as f:
    scene_xml = f.read()

# 2. Find robot model from schema
robot_folder = "franka_emika_panda"  # from schema.json

# 3. Change to robot directory
os.chdir(menagerie_dir / robot_folder)

# 4. Load from string (NOT from path!)
model = mujoco.MjModel.from_xml_string(scene_xml)

# 5. Restore original directory
os.chdir(original_cwd)
```

**Why this works:**
- The `<include file="panda.xml"/>` in our scene gets resolved relative to the current directory
- When we `chdir` to `mujoco_menagerie/franka_emika_panda/`, MuJoCo can find:
  - `panda.xml` (the robot definition)
  - `assets/` folder (meshes, textures)
  - All relative paths in `panda.xml`

**Key Functions:**

#### a. `launch_scene(scene_file, schema_file)`

**Steps:**
1. **Read Schema**: Extracts robot model name from `schema.json`
   ```python
   robot_folder = comp["properties"]["model"]  # e.g., "franka_emika_panda"
   ```

2. **Read Scene XML**: Loads the generated MJCF as a string
   ```python
   with open(scene_path) as f:
       scene_xml = f.read()
   ```

3. **Change Directory**: Navigates to robot folder
   ```python
   menagerie_dir = "../mujoco_menagerie/franka_emika_panda"
   os.chdir(menagerie_dir)
   ```

4. **Load Model**: Uses `from_xml_string()` to parse MJCF
   ```python
   model = mujoco.MjModel.from_xml_string(scene_xml)
   data = mujoco.MjData(model)
   ```

5. **Restore Directory**: Returns to original working directory
   ```python
   os.chdir(original_cwd)
   ```

6. **Launch Viewer**: Creates passive viewer for real-time visualization
   ```python
   with mujoco.viewer.launch_passive(model, data) as viewer:
       while viewer.is_running():
           mujoco.mj_step(model, data)
           viewer.sync()
   ```

**Usage:**
```bash
python mujoco_launcher.py [scene_file]
```
Default: `output_scene.xml` with `schema.json`

---

## Workflow

### Step 1: Define Scene (schema.json)
```json
{
  "scene": {
    "name": "pick_and_place_workcell"
  },
  "components": [
    {
      "id": "floor",
      "type": "floor",
      "position": [0, 0, 0]
    },
    {
      "id": "ROBOT-001",
      "type": "robot",
      "position": [1.5, -0.8, 0.85],
      "properties": {
        "model": "franka_emika_panda"
      }
    },
    {
      "id": "TABLE-001",
      "type": "table",
      "position": [1.5, 0, 0.85],
      "properties": {
        "dimensions": [0.4, 0.3, 0.025]
      }
    }
  ]
}
```

### Step 2: Generate MJCF
```bash
python compile_scene.py
```
**Output:** `output_scene.xml`

### Step 3: Launch Simulator
```bash
python mujoco_launcher.py
```
**Result:** MuJoCo viewer opens with your scene

---

## Supported Components

| Type       | Description                          | Required Properties | Optional Properties |
|------------|--------------------------------------|---------------------|---------------------|
| `floor`    | Ground plane                         | `position`          | `size`, `rgba`      |
| `robot`    | Robot arm from menagerie             | `position`, `model` | -                   |
| `table`    | Work surface with legs               | `position`          | `dimensions`, `rgba`|
| `conveyor` | Conveyor belt with rails             | `position`          | `dimensions`, `rgba`|
| `bin`      | Storage bin                          | `position`          | `dimensions`, `rgba`|
| `sensor`   | Camera with mounting pole            | `position`          | `fov_deg`           |
| `object`   | Manipulable object (with freejoint)  | `position`, `parent`| `dimensions`, `rgba`, `mass` |

---

## Key Design Decisions

### 1. **Robot Includes at Root Level**
Robot XML files are complete MuJoCo models. They must be included at the root `<mujoco>` level, not nested inside bodies.

### 2. **Freejoint Constraint**
MuJoCo requires freejoint bodies to be direct children of `<worldbody>`. The compiler automatically promotes objects with freejoint to top level and calculates their absolute positions.

### 3. **Path Handling**
The `from_xml_string()` + `chdir()` pattern is essential for robot models with relative asset paths. Using `from_xml_path()` would fail to resolve includes correctly.

### 4. **Visual Enhancements**
- **Table legs**: Automatically generated at corners for realism
- **Conveyor rails**: Yellow guides show the transport path
- **Camera boxes**: Red boxes mark camera positions for debugging

---

## Troubleshooting

### Error: "XML Error: Error opening file 'panda.xml'"
**Cause:** Directory not changed before loading model  
**Fix:** Ensure `mujoco_launcher.py` is reading schema correctly and changing to robot directory

### Error: "Schema violation: unrecognized element 'compiler'"
**Cause:** Robot include is nested inside a body instead of at root level  
**Fix:** Regenerate scene with `python compile_scene.py`

### Error: "free joint can only be used on top level"
**Cause:** Object with freejoint is nested inside another body  
**Fix:** The compiler should handle this automatically, but verify object's parent in schema

---

## Extending the System

### Adding New Robot Models
1. Add to robot name mapping in `compile_scene.py`:
   ```python
   robot_name_map = {
       "franka_emika_panda": "panda",
       "your_robot_name": "robot_file",  # Add here
   }
   ```

2. Ensure robot exists in `../mujoco_menagerie/your_robot_name/`

### Adding New Component Types
1. Add defaults to `DEFAULTS` dict
2. Add handling in the component processing loop:
   ```python
   elif c_type == "new_type":
       # Build geometry here
       pass
   ```

---

## Dependencies

- `mujoco` - Physics engine and viewer
- `numpy` - Numerical operations
- `xml.etree.ElementTree` - XML generation
- `xml.dom.minidom` - XML formatting

---

## Author Notes

This system demonstrates a clean separation between scene definition (JSON) and physics representation (MJCF). The key insight is that robot models from mujoco_menagerie can be dynamically included by manipulating the working directory during model loading, allowing for flexible scene composition without modifying robot files.
