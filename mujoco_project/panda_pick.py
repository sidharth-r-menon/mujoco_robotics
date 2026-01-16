import time
import numpy as np
import mujoco
import mujoco.viewer
import os

# --- PATH LOGIC ---
curr_dir = os.path.dirname(os.path.abspath(__file__))
panda_dir = os.path.join(curr_dir, "mujoco_menagerie", "franka_emika_panda")

SCENE_XML = f"""
<mujoco model="panda_pick_scene">
  <include file="panda.xml"/>
  <statistic center="0.4 0 0.5" extent="1.0"/>
  <worldbody>
    <light pos="0 0 2" dir="0 0 -1" directional="true"/>
    <geom name="floor" type="plane" size="0 0 0.05" rgba="0.9 0.9 0.9 1"/>
    <body name="table" pos="0.5 0 0.4">
      <geom type="box" size="0.25 0.4 0.4" rgba="0.3 0.3 0.3 1"/>
    </body>
    <body name="box" pos="0.6 0 0.85">
      <freejoint/>
      <geom type="box" size="0.02 0.02 0.02" rgba="1 0 0 1" mass="0.1" friction="1 0.5 0.5"/>
    </body>
  </worldbody>
</mujoco>
"""

# -------------------------------------------------------------------------
# 1. IK UTILITY (TARGETING A BODY)
# -------------------------------------------------------------------------
def solve_ik_body(model, data, target_pos, body_id, damping=1e-2):
    # Get current position of the body's center of mass
    current_pos = data.xpos[body_id]
    error = target_pos - current_pos
    
    # Calculate Jacobian for the body's center of mass
    jacp = np.zeros((3, model.nv))
    mujoco.mj_jacBody(model, data, jacp, None, body_id)
    
    # Solve for joint velocities
    jtj = jacp.T @ jacp + damping * np.eye(model.nv)
    qvel = np.linalg.solve(jtj, jacp.T @ error)
    return qvel

# -------------------------------------------------------------------------
# 2. MAIN LOOP
# -------------------------------------------------------------------------
def run():
    original_cwd = os.getcwd()
    try:
        os.chdir(panda_dir)
        model = mujoco.MjModel.from_xml_string(SCENE_XML)
        data = mujoco.MjData(model)
        print("Model loaded successfully!")
    finally:
        os.chdir(original_cwd)

    # --- BODY DISCOVERY ---
    # We look for a body that represents the end effector.
    ee_body_name = None
    possible_bodies = ["hand", "link7", "right_hand"]
    
    for name in possible_bodies:
        try:
            model.body(name)
            ee_body_name = name
            print(f"Using Body: {ee_body_name}")
            break
        except ValueError:
            continue
            
    if ee_body_name is None:
        available_bodies = [model.body(i).name for i in range(model.nbody)]
        print(f"Available bodies: {available_bodies}")
        raise RuntimeError("Could not find a valid gripper body.")

    ee_id = model.body(ee_body_name).id
    
    # Control Parameters
    arm_actuators = list(range(7))
    gripper_actuator = 7 
    box_pos = np.array([0.6, 0.0, 0.85])
    drop_pos = np.array([0.4, 0.3, 1.0])
    dt = model.opt.timestep

    with mujoco.viewer.launch_passive(model, data) as viewer:
        state = 0
        state_timer = None
        
        while viewer.is_running():
            step_start = time.time()
            
            # Current End Effector position
            ee_pos = data.xpos[ee_id]

            if state == 0:  # Approach (Higher offset for body-based IK)
                target = box_pos + [0, 0, 0.15]
                gripper = 255
                if np.linalg.norm(target - ee_pos) < 0.04: state = 1
            elif state == 1:  # Grasp
                target = box_pos + [0, 0, 0.05]
                gripper = 0
                if state_timer is None: state_timer = time.time()
                if time.time() - state_timer > 1.2:
                    state = 2
                    state_timer = None
            elif state == 2:  # Lift
                target = box_pos + [0, 0, 0.3]
                gripper = 0
                if np.linalg.norm(target - ee_pos) < 0.05: state = 3
            elif state == 3:  # Move
                target = drop_pos
                gripper = 0
                if np.linalg.norm(target - ee_pos) < 0.05:
                    state = 4
                    state_timer = time.time()
            else:  # Release
                target = drop_pos
                gripper = 255
                if time.time() - state_timer > 1.5:
                    mujoco.mj_resetData(model, data)
                    state = 0
                    state_timer = None

            # Apply IK targeting the body center
            qvel = solve_ik_body(model, data, target, ee_id)
            for i, aid in enumerate(arm_actuators):
                data.ctrl[aid] = data.qpos[i] + qvel[i] * dt * 5.0
            
            data.ctrl[gripper_actuator] = gripper

            mujoco.mj_step(model, data)
            viewer.sync()

            sleep = dt - (time.time() - step_start)
            if sleep > 0:
                time.sleep(sleep)

if __name__ == "__main__":
    run()