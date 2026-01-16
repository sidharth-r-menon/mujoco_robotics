import time
import numpy as np
import mujoco
import mujoco.viewer
import os

# ------------------------------------------------------------
# PATH LOGIC (MATCHES YOUR WORKING CODE)
# ------------------------------------------------------------
curr_dir = os.path.dirname(os.path.abspath(__file__))
panda_dir = os.path.join(curr_dir, "mujoco_menagerie", "franka_emika_panda")

# ------------------------------------------------------------
# SCENE XML
# ------------------------------------------------------------
SCENE_XML = """
<mujoco model="panda_pick_place">
  <include file="panda.xml"/>
  <statistic center="0.4 0 0.5" extent="1.0"/>

  <worldbody>
    <light pos="0 0 2" dir="0 0 -1" directional="true"/>
    <geom type="plane" size="0 0 0.05" rgba="0.9 0.9 0.9 1"/>

    <!-- Table -->
    <body name="table" pos="0.6 0 0.2">
      <geom type="box" size="0.3 0.4 0.2" rgba="0.3 0.3 0.3 1"/>
    </body>

    <!-- Object -->
    <body name="box" pos="0.6 0 0.43">
      <freejoint/>
      <geom type="box"
            size="0.025 0.025 0.025"
            mass="0.05"
            friction="2 1 1"
            rgba="1 0 0 1"/>
    </body>
  </worldbody>
</mujoco>
"""

# ------------------------------------------------------------
# CLASSICAL DIFFERENTIAL IK (ARM-ONLY SOLUTION)
# ------------------------------------------------------------
def solve_ik_body(model, data, target_pos, body_id, damping=1e-2):
    current_pos = data.xpos[body_id]
    error = target_pos - current_pos

    # MuJoCo REQUIRES full nv-sized Jacobian
    jacp_full = np.zeros((3, model.nv))
    mujoco.mj_jacBody(model, data, jacp_full, None, body_id)

    # Use only arm DOFs (first 7)
    jacp = jacp_full[:, :7]

    jtj = jacp.T @ jacp + damping * np.eye(7)
    qvel = np.linalg.solve(jtj, jacp.T @ error)
    return qvel

# ------------------------------------------------------------
# MAIN
# ------------------------------------------------------------
def run():
    original_cwd = os.getcwd()
    try:
        os.chdir(panda_dir)
        model = mujoco.MjModel.from_xml_string(SCENE_XML)
        data = mujoco.MjData(model)
        print("Model loaded successfully")
    finally:
        os.chdir(original_cwd)

    ee_id = model.body("hand").id
    box_id = model.body("box").id

    arm_actuators = list(range(7))
    gripper_actuator = 7
    dt = model.opt.timestep

    state = 0
    timer = None

    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            step_start = time.time()

            ee_pos = data.xpos[ee_id]
            box_pos = data.xpos[box_id]

            # ---------------- FSM ----------------
            if state == 0:  # hover
                target = box_pos + np.array([0, 0, 0.20])
                gripper = 255
                if np.linalg.norm(target - ee_pos) < 0.03:
                    state = 1

            elif state == 1:  # descend
                target = box_pos + np.array([0, 0, 0.08])
                gripper = 255
                if np.linalg.norm(target - ee_pos) < 0.015:
                    state = 2
                    timer = time.time()

            elif state == 2:  # grasp
                target = box_pos + np.array([0, 0, 0.08])
                gripper = 0
                if time.time() - timer > 1.0:
                    state = 3

            elif state == 3:  # lift
                target = box_pos + np.array([0, 0, 0.30])
                gripper = 0
                if ee_pos[2] > 0.9:
                    state = 4

            elif state == 4:  # move
                target = np.array([0.4, 0.3, 0.9])
                gripper = 0
                if np.linalg.norm(target - ee_pos) < 0.04:
                    state = 5
                    timer = time.time()

            else:  # release
                target = np.array([0.4, 0.3, 0.9])
                gripper = 255
                if time.time() - timer > 1.2:
                    mujoco.mj_resetData(model, data)
                    state = 0
                    timer = None

            # -------------------------------------

            qvel = solve_ik_body(model, data, target, ee_id)

            for i in arm_actuators:
                data.ctrl[i] = data.qpos[i] + qvel[i] * dt * 5.0

            data.ctrl[gripper_actuator] = gripper

            mujoco.mj_step(model, data)
            viewer.sync()

            sleep = dt - (time.time() - step_start)
            if sleep > 0:
                time.sleep(sleep)

if __name__ == "__main__":
    run()
