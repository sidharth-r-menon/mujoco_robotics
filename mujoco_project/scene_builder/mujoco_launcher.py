#!/usr/bin/env python3
"""
MuJoCo Simulator Launcher

Launches the compiled scene in MuJoCo viewer with proper path handling.
"""

import time
import os
import sys
from pathlib import Path
import mujoco
import mujoco.viewer


def launch_scene(scene_file="output_scene.xml", schema_file="schema.json"):
    """
    Launch MuJoCo simulator with the generated scene.
    
    Uses EXACT same pattern as panda_pick.py:
    - Read schema to find robot model
    - Read scene XML as string
    - Change directory to robot folder  
    - Use from_xml_string() to load model
    - Restore directory
    """
    import json
    import re
    
    script_dir = Path(__file__).parent
    scene_path = script_dir / scene_file
    schema_path = script_dir / schema_file
    
    if not scene_path.exists():
        print(f"Error: Scene file not found: {scene_path}")
        print("Please run compile_scene.py first.")
        sys.exit(1)
    
    print(f"Loading scene: {scene_path}")
    
    # Read the XML content first (before any directory changes)
    with open(scene_path) as f:
        scene_xml = f.read()
    
    # Read schema to find robot model
    robot_folder = None
    if schema_path.exists():
        with open(schema_path) as f:
            schema = json.load(f)
            
        # Find robot component
        for comp in schema.get("components", []):
            if comp.get("type") == "robot":
                robot_folder = comp.get("properties", {}).get("model", "franka_emika_panda")
                print(f"Found robot in schema: {robot_folder}")
                break
    
    # Change directory to robot folder if needed (EXACT same pattern as panda_pick.py)
    original_cwd = os.getcwd()
    try:
        if robot_folder:
            # Build path to robot directory (same as panda_pick.py)
            menagerie_dir = script_dir.parent / "mujoco_menagerie" / robot_folder
            menagerie_dir_str = str(menagerie_dir)
            
            if os.path.exists(menagerie_dir_str):
                print(f"Changing directory to: {menagerie_dir_str}")
                os.chdir(menagerie_dir_str)
                print(f"✓ Current directory: {os.getcwd()}")
            else:
                print(f"Warning: Robot folder not found: {menagerie_dir_str}")
        
        # Use from_xml_string() - EXACT same as panda_pick.py!
        model = mujoco.MjModel.from_xml_string(scene_xml)
        
        data = mujoco.MjData(model)
        print("Model loaded successfully!")
        print(f"  Bodies: {model.nbody}")
        print(f"  Joints: {model.njnt}")
        print(f"  Actuators: {model.nu}")
        
    finally:
        os.chdir(original_cwd)
    
    print("\nLaunching MuJoCo viewer...")
    print("Press Ctrl+C to exit.")
    
    dt = model.opt.timestep
    
    with mujoco.viewer.launch_passive(model, data) as viewer:
        try:
            while viewer.is_running():
                step_start = time.time()
                
                mujoco.mj_step(model, data)
                viewer.sync()
                
                # Real-time simulation
                elapsed = time.time() - step_start
                if elapsed < dt:
                    time.sleep(dt - elapsed)
                    
        except KeyboardInterrupt:
            print("\nShutting down...")


if __name__ == "__main__":
    scene_file = sys.argv[1] if len(sys.argv) > 1 else "output_scene.xml"
    launch_scene(scene_file)
