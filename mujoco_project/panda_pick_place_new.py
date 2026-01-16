import time
import numpy as np
import mujoco
import mujoco.viewer
import os
import re

# ------------------------------------------------------------
# PATH LOGIC (Matching your provided structure)
# ------------------------------------------------------------
curr_dir = os.path.dirname(os.path.abspath(__file__))
panda_dir = os.path.join(curr_dir, "mujoco_menagerie", "franka_emika_panda")
panda_xml_path = os.path.join(panda_dir, "panda.xml")

def get_panda_components(path):
    """Extracts necessary tags to avoid XML nesting errors."""
    with open(path, 'r') as f:
        xml = f.read()
    
    def extract(tag):
        match = re.search(f'<{tag}[^>]*>(.*)</{tag}>', xml, re.DOTALL)
        return match.group(1) if match else ""

    return {
        "default": extract("default"),
        "asset": extract("asset"),
        "worldbody": extract("worldbody"),
        "contact": extract("contact"),
        "tendon": extract("tendon"),
        "actuator": extract("actuator")
    }

# ------------------------------------------------------------
# PREPARE THE MERGED XML
# ------------------------------------------------------------
panda = get_panda_components(panda_xml_path)

# Using 'meshdir' with the absolute path to fix the STL loading error
FULL_SCENE_XML = f"""
<mujoco model="panda_pick_place">
  <compiler angle="degree" meshdir="{panda_dir}" />
  
  <statistic center="0.4 0 0.5" extent="1.0"/>
  
  <visual>
    <headlight diffuse="0.6 0.6 0.6" ambient="0.3 0.3 0.3"/>
    <global azimuth="120" elevation="-20"/>
  </visual>

  <default>
    {panda['default']}
  </default>

  <asset>
    {panda['asset']}
  </asset>

  <worldbody>
    <light pos="0 0 2" dir="0 0 -1" directional="true"/>
    <geom type="plane" size="1.5 1.5 0.05" rgba="0.9 0.9 0.9 1"/>

    <body name="pedestal" pos="0 0 0.2">
      <geom type="cylinder" size="0.12 0.2" rgba="0.4 0.4 0.4 1"/>
      <body name="robot_mount" pos="0 0 0.2">
         {panda['worldbody']}
      </body>
    </body>

    <body name="table" pos="0.6 0 0.2">
      <geom type="box" size="0.3 0.4 0.2" rgba="0.3 0.3 0.3 1"/>
    </body>

    <body name="box" pos="0.6 0 0.425">
      <freejoint/>
      <geom type="box" size="0.02 0.02 0.02" rgba="1 0 0 1" mass="0.05" friction="2 1 1"/>
    </body>

    <body name="goal" mocap="true" pos="0.6 0 0.6">
      <geom type="sphere" size="0.01" rgba="0 1 0 0.5" contype="0" conaffinity="0"/>
    </body>
  </worldbody>

  <contact>{panda['contact']}</contact>
  <tendon>{panda['tendon']}</tendon>
  <actuator>{panda['actuator']}</actuator>
</mujoco>
"""

# ------------------------------------------------------------
# IK UTILITIES
# ------------------------------------------------------------
def solve_ik(model, data, target_pos, target_quat, body_id):
    current_pos = data.xpos[body_id]
    pos_error = target_pos - current_pos

    # Orientation error
    current_quat = np.zeros(4)
    mujoco.mju_mat2Quat(current_quat, data.xmat[body_id])
    neg_quat = np.zeros(4)
    mujoco.mju_negQuat(neg_quat, current_quat)
    error_quat = np.zeros(4)
    mujoco.mju_mulQuat(error_quat, target_quat, neg_quat)
    ori_error = np.zeros(3)
    mujoco.mju_quat2Vel(ori_error, error_quat, 1.0)

    # 6D error
    error = np.concatenate([pos_error, ori_error])
    
    # Jacobian
    jac = np.zeros((6, model.nv))
    mujoco.mj_jacBody(model, data, jac[:3], jac[3:], body_id)
    
    # Arm only (first 7 DOFs)
    jac_arm = jac[:, :7]
    qvel = np.linalg.pinv(jac_arm) @ error
    return qvel

# ------------------------------------------------------------
# MAIN LOOP
# ------------------------------------------------------------
def run():
    try:
        model = mujoco.MjModel.from_xml_string(FULL_SCENE_XML)
        data = mujoco.MjData(model)
        print("Model loaded successfully!")
    except Exception as e:
        print(f"Error loading model: {e}")
        return

    # IDs
    try:
        ee_id = model.body("hand").id
    except:
        ee_id = model.body("link7").id
        
    box_id = model.body("box").id
    mocap_id = model.body("goal").mocapid[0]
    dt = model.opt.timestep
    down_quat = np.array([0, 0.7071, 0.7071, 0]) 

    state = 0
    timer = None
    smoothed_target = data.xpos[ee_id].copy()

    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            step_start = time.time()
            ee_pos = data.xpos[ee_id]
            box_pos = data.xpos[box_id]

            # --- FSM ---
            if state == 0:  # Approach
                target = box_pos + [0, 0, 0.20]
                gripper = 255
                if np.linalg.norm(target - ee_pos) < 0.02: state = 1
            elif state == 1:  # Lower
                target = box_pos + [0, 0, 0.11] 
                gripper = 255
                if np.linalg.norm(target - ee_pos) < 0.01:
                    state = 2
                    timer = time.time()
            elif state == 2:  # Grasp
                target = box_pos + [0, 0, 0.11]
                gripper = 0
                if time.time() - timer > 1.2: state = 3
            elif state == 3:  # Lift
                target = box_pos + [0, 0, 0.35]
                gripper = 0
                if ee_pos[2] > 0.65: state = 4
            elif state == 4:  # Move
                target = np.array([0.4, 0.3, 0.7])
                gripper = 0
                if np.linalg.norm(target - ee_pos) < 0.04:
                    state = 5
                    timer = time.time()
            else:  # Release
                target = np.array([0.4, 0.3, 0.7])
                gripper = 255
                if time.time() - timer > 1.2:
                    mujoco.mj_resetData(model, data)
                    state = 0
                    timer = None

            # Exponential Smoothing
            alpha = 0.05 
            smoothed_target = alpha * target + (1 - alpha) * smoothed_target
            data.mocap_pos[mocap_id] = smoothed_target

            # Solve IK
            qvel = solve_ik(model, data, smoothed_target, down_quat, ee_id)

            # Control
            for i in range(7):
                data.ctrl[i] = data.qpos[i] + qvel[i] * dt * 8.0
            data.ctrl[7] = gripper

            mujoco.mj_step(model, data)
            viewer.sync()
            
            elapsed = time.time() - step_start
            if elapsed < dt:
                time.sleep(dt - elapsed)

if __name__ == "__main__":
    run()