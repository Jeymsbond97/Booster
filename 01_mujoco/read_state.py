"""Step the K1 simulation without control and print what the robot "feels".

This is the raw material a locomotion policy observes (see Table I of the
Booster Gym paper): torso height, gravity vector in the body frame (from the
IMU orientation), angular velocity (gyro), joint positions and velocities,
and which geoms touch the ground.

Run from the repository root:
    python 01_mujoco/read_state.py
"""
from pathlib import Path

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parent.parent
MODEL_PATH = ROOT / "assets/K1/K1_22dof.xml"

SIM_SECONDS = 2.0
PRINT_EVERY = 0.25  # seconds

model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
data = mujoco.MjData(model)

# The free joint occupies qpos[0:7] (xyz + quaternion) and qvel[0:6].
# The 22 hinge joints follow, in actuator order.
joint_names = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, j) for j in range(1, model.njnt)]


def gravity_in_body_frame(quat_wxyz):
    """Rotate world gravity (0, 0, -1) into the torso frame. Upright robot -> (0, 0, -1)."""
    rot = np.zeros(9)
    mujoco.mju_quat2Mat(rot, quat_wxyz)
    return rot.reshape(3, 3).T @ np.array([0.0, 0.0, -1.0])


def ground_contacts():
    ground = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "ground")
    bodies = set()
    for c in data.contact[: data.ncon]:
        other = c.geom2 if c.geom1 == ground else c.geom1 if c.geom2 == ground else None
        if other is not None:
            bodies.add(mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, model.geom_bodyid[other]))
    return sorted(bodies)


steps_per_print = int(PRINT_EVERY / model.opt.timestep)
for step in range(int(SIM_SECONDS / model.opt.timestep) + 1):
    if step % steps_per_print == 0:
        orientation = data.sensor("orientation").data  # quaternion (w, x, y, z)
        gyro = data.sensor("angular-velocity").data
        q = data.qpos[7:]
        dq = data.qvel[6:]
        print(f"t={data.time:4.2f}s  height={data.qpos[2]:.3f} m")
        print(f"  gravity (body frame) = {np.round(gravity_in_body_frame(orientation), 3)}")
        print(f"  angular velocity     = {np.round(gyro, 3)}")
        print(f"  |q|max = {np.abs(q).max():.3f} rad ({joint_names[np.abs(q).argmax()]})"
              f"   |dq|max = {np.abs(dq).max():.3f} rad/s")
        print(f"  ground contacts      = {ground_contacts() or 'none'}")
    mujoco.mj_step(model, data)
