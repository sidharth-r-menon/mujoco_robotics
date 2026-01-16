# MuJoCo Robotics Project

A collection of MuJoCo-based robotics simulation tools and examples.

## Projects

### 🏗️ [Scene Builder](scene_builder/README.md)
**JSON-to-MJCF Pipeline** - Automatically generate MuJoCo simulation scenes from declarative JSON schemas.

Build robotic workcells by defining components in JSON and compiling them into MuJoCo XML format.

```bash
cd scene_builder
python compile_scene.py
python mujoco_launcher.py
```

See [scene_builder/README.md](scene_builder/README.md) for detailed documentation.

---

### 📦 Other Contents

- **mujoco_menagerie/** - Robot model library (Franka Panda, UR5e, etc.)
- **panda_pick.py** - Pick and place example with Franka Panda
- **panda_pick_place_*.py** - Various manipulation demos

---

## Requirements

```bash
pip install mujoco numpy
```

## Quick Start

1. Define your scene in `scene_builder/schema.json`
2. Generate MJCF: `python scene_builder/compile_scene.py`
3. Launch simulator: `python scene_builder/mujoco_launcher.py`

## Repository Structure

```
mujoco_project/
├── scene_builder/          # JSON → MJCF pipeline (see README)
│   ├── compile_scene.py    # Scene generator
│   ├── mujoco_launcher.py  # Simulator launcher
│   ├── schema.json         # Scene definition
│   └── README.md          # Detailed documentation
├── mujoco_menagerie/       # Robot models library
├── panda_*.py             # Example scripts
└── README.md              # This file
```

---

**For detailed scene builder documentation, see [scene_builder/README.md](scene_builder/README.md)**
