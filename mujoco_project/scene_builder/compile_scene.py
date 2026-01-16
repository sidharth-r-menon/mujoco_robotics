#!/usr/bin/env python3
"""
Scene Compiler: JSON Schema -> MJCF

Usage:
    python compile_scene.py [schema_file]
"""

import json
import sys
import xml.etree.ElementTree as ET
from xml.dom import minidom
from pathlib import Path
import numpy as np

# --- 1. THE DEFAULTS LIBRARY ---
DEFAULTS = {
    "table": {"size": [0.3, 0.2, 0.025], "rgba": [0.6, 0.6, 0.6, 1], "height": 0.8},
    "conveyor": {"size": [0.2, 0.4, 0.025], "rgba": [0.2, 0.2, 0.2, 1], "height": 0.8},
    "bin": {"size": [0.15, 0.15, 0.1], "rgba": [0.2, 0.8, 0.2, 0.5]},
    "floor": {"size": [5, 5, 0.1], "rgba": [0.9, 0.9, 0.9, 1]},
    "robot": {"offset": [0, 0, 0]}
}

def create_element(tag, **attribs):
    """Helper to create XML elements cleaner"""
    str_attribs = {k: " ".join(map(str, v)) if isinstance(v, (list, tuple, np.ndarray)) else str(v) 
                   for k, v in attribs.items() if v is not None}
    return ET.Element(tag, str_attribs)

def add_legs(parent_body, surface_pos, surface_size):
    """Automatically adds 4 legs to a table/conveyor body"""
    leg_radius = 0.03
    table_z = surface_pos[2]
    leg_len = table_z 
    leg_z = -table_z / 2  # Relative to table center
    
    dx = surface_size[0] * 0.8
    dy = surface_size[1] * 0.8
    
    corners = [(dx, dy), (dx, -dy), (-dx, dy), (-dx, -dy)]
    
    for (cx, cy) in corners:
        leg = create_element("geom", 
                             type="cylinder", 
                             size=[leg_radius, leg_len/2], 
                             pos=[cx, cy, leg_z], 
                             rgba=[0.3, 0.3, 0.3, 1])
        parent_body.append(leg)

def build_mujoco_xml(json_schema, output_file="output_scene.xml"):
    root = create_element("mujoco", model=json_schema["scene"]["name"])
    
    # Compiler & Options
    root.append(create_element("compiler", angle="degree", coordinate="local", meshdir="meshes/"))
    root.append(create_element("option", integrator="RK4", timestep="0.01", gravity=[0, 0, -9.81]))

    # Include Robot
    robot_model = None
    for comp in json_schema["components"]:
        if comp["type"] == "robot":
            robot_model = comp.get("properties", {}).get("model", "franka_emika_panda")
            
            # Map robot model name to XML filename
            robot_name_map = {
                "franka_emika_panda": "panda",
                "franka_fr3": "fr3",
                "universal_robots_ur5e": "ur5e",
                "unitree_go1": "go1",
            }
            
            xml_name = robot_name_map.get(robot_model, robot_model.split("_")[-1])
            robot_xml = f"{xml_name}.xml"
            
            # Add comment for launcher
            root.append(ET.Comment(f" ROBOT_FOLDER:{robot_model} "))
            root.append(create_element("include", file=robot_xml))
            
            # Add keyframe for Panda
            if "panda" in robot_model:
                keyframe = create_element("keyframe")
                keyframe.append(create_element("key", name="pick_ready", qpos="0 -0.785 0 -2.356 0 1.571 0.785 0.04 0.04"))
                root.append(keyframe)
            break
            
    worldbody = create_element("worldbody")
    root.append(worldbody)
    worldbody.append(create_element("light", pos=[0, 0, 3], dir=[0, 0, -1], diffuse=[0.8, 0.8, 0.8]))

    # Pre-scan for finding parents and targets
    comp_map = {c["id"]: c for c in json_schema["components"]}
    # Find the first table to serve as the default camera target
    default_target = "world"
    for c in json_schema["components"]:
        if c["type"] == "table":
            default_target = f"{c['id']}_target_body"
            break

    for comp in json_schema["components"]:
        c_type = comp["type"]
        c_id = comp["id"]
        props = comp.get("properties", {})
        pos = comp.get("position", [0, 0, 0])
        
        # 1. FLOOR
        if c_type == "floor":
            size = props.get("size", DEFAULTS["floor"]["size"])
            rgba = props.get("rgba", DEFAULTS["floor"]["rgba"])
            geom = create_element("geom", name=c_id, type="plane", size=size, rgba=rgba)
            worldbody.append(geom)

        # 2. ROBOT MOUNT
        elif c_type == "robot":
            mount_body = create_element("body", name=f"{c_id}_mount", pos=pos)
            pedestal = create_element("geom", type="cylinder", size=[0.15, 0.05], rgba=[0.2, 0.2, 0.2, 1])
            mount_body.append(pedestal)
            worldbody.append(mount_body)

        # 3. TABLES / CONVEYORS
        elif c_type in ["table", "conveyor"]:
            dim = props.get("dimensions", DEFAULTS[c_type]["size"])
            rgba = props.get("rgba", DEFAULTS[c_type]["rgba"])
            
            body = create_element("body", name=c_id, pos=pos)
            
            # Main Surface
            body.append(create_element("geom", type="box", size=dim, rgba=rgba))

            # --- VISUAL FIX 1: Add Yellow Rails for Conveyors ---
            if c_type == "conveyor":
                rail_width = 0.02
                # Calculate rail position: Edge of the conveyor width
                rail_x = dim[0] - (rail_width / 2) 
                # Place rail slightly higher than surface
                rail_z = dim[2] # sits on top
                rail_size = [rail_width, dim[1], 0.03] # Full length (Y), slight height (Z)

                # Left Rail (Yellow)
                body.append(create_element("geom", type="box", size=rail_size, pos=[rail_x, 0, rail_z], rgba=[0.9, 0.9, 0, 1]))
                # Right Rail (Yellow)
                body.append(create_element("geom", type="box", size=rail_size, pos=[-rail_x, 0, rail_z], rgba=[0.9, 0.9, 0, 1]))

            # --- Target Logic (Table Only) ---
            if c_type == "table":
                 target_name = f"{c_id}_target_body"
                 site_z = dim[2] + 0.001
                 target_body = create_element("body", name=target_name, pos=[0, 0, site_z])
                 target_body.append(create_element("site", name=f"{c_id}_visual", size="0.01", rgba=[1, 0, 0, 1]))
                 body.append(target_body)
            
            add_legs(body, pos, dim)
            worldbody.append(body)
            
        # 4. SENSORS (Cameras)
        elif c_type == "sensor":
            pole_height = pos[2]
            # Create mount body at floor level
            mount_body = create_element("body", name=f"{c_id}_mount", pos=[pos[0], pos[1], 0])
            
            # The Pole
            pole = create_element("geom", type="cylinder", size=[0.04, pole_height/2], pos=[0, 0, pole_height/2], rgba=[0.1, 0.1, 0.1, 1])
            mount_body.append(pole)
            
            # --- VISUAL FIX 2: Add Visible Camera Box ---
            # A red box at the top of the pole to represent the camera body
            cam_box_size = [0.05, 0.05, 0.1]
            mount_body.append(create_element("geom", type="box", size=cam_box_size, pos=[0, 0, pole_height], rgba=[0.8, 0.2, 0.2, 1]))

            # The Actual Camera Sensor (Invisible logic)
            camera = create_element("camera", name=c_id, pos=[0, 0, pole_height], target=default_target, fovy=props.get("fov_deg", 45))
            mount_body.append(camera)
            worldbody.append(mount_body)

        # 5. OBJECTS
        elif c_type == "object":
            parent_id = comp.get("parent", "world")
            final_pos = list(pos)
            
            if parent_id in comp_map:
                parent = comp_map[parent_id]
                parent_pos = parent.get("position", [0,0,0])
                final_pos[0] += parent_pos[0]
                final_pos[1] += parent_pos[1]
                
                if parent["type"] in ["table", "conveyor"]:
                    p_dim = parent.get("properties", {}).get("dimensions", DEFAULTS["table"]["size"])
                    final_pos[2] = parent_pos[2] + p_dim[2] + pos[2] 
            
            body = create_element("body", name=c_id, pos=final_pos)
            body.append(create_element("freejoint"))
            
            dim = props.get("dimensions", [0.02, 0.02, 0.02])
            rgba = props.get("rgba", [1, 0.5, 0, 1])
            mass = props.get("mass", 0.1)
            
            body.append(create_element("geom", type="box", size=dim, rgba=rgba, mass=mass))
            worldbody.append(body)

    # Output
    xml_str = minidom.parseString(ET.tostring(root)).toprettyxml(indent="    ")
    with open(output_file, "w") as f:
        f.write(xml_str)
    print(f"✓ Successfully generated {output_file}")
    return robot_model

if __name__ == "__main__":
    script_dir = Path(__file__).parent
    schema_file = sys.argv[1] if len(sys.argv) > 1 else "schema.json"
    schema_path = script_dir / schema_file
    output_path = script_dir / "output_scene.xml"
    
    try:
        print(f"Loading schema: {schema_path}")
        with open(schema_path, "r") as f:
            data = json.load(f)
        build_mujoco_xml(data, str(output_path))
        print(f"\nTo launch simulator: python mujoco_launcher.py")
    except FileNotFoundError:
        print(f"Error: {schema_path} not found.")
        sys.exit(1)
    except json.JSONDecodeError as e:
        print(f"Error: Invalid JSON in {schema_path}: {e}")
        sys.exit(1)
