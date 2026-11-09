"""Print the structure of an MJCF model: bodies, joints, actuators, sensors, cameras.

Run from the repository root:
    python 01_mujoco/inspect_model.py
    python 01_mujoco/inspect_model.py assets/K1/K1_22dof_parallel.xml
"""
import sys
from pathlib import Path

import mujoco

ROOT = Path(__file__).resolve().parent.parent
DEFAULT_MODEL = ROOT / "assets/K1/K1_22dof.xml"

JOINT_TYPES = {
    int(mujoco.mjtJoint.mjJNT_FREE): "free",
    int(mujoco.mjtJoint.mjJNT_BALL): "ball",
    int(mujoco.mjtJoint.mjJNT_SLIDE): "slide",
    int(mujoco.mjtJoint.mjJNT_HINGE): "hinge",
}


def name(model, obj_type, idx):
    return mujoco.mj_id2name(model, obj_type, idx) or f"<unnamed {idx}>"


def print_bodies(model):
    print(f"\nBODIES ({model.nbody}) — kinematic tree, mass in kg")
    for b in range(model.nbody):
        depth, parent = 0, b
        while parent != 0:
            parent = model.body_parentid[parent]
            depth += 1
        print(f"  {'  ' * depth}{name(model, mujoco.mjtObj.mjOBJ_BODY, b)}  [{model.body_mass[b]:.3f}]")


def print_joints(model):
    print(f"\nJOINTS ({model.njnt}) — type, axis, range (rad), armature")
    for j in range(model.njnt):
        jtype = JOINT_TYPES[int(model.jnt_type[j])]
        jname = name(model, mujoco.mjtObj.mjOBJ_JOINT, j)
        if jtype == "free":
            print(f"  {jname:32s} free (7 qpos: xyz + quaternion, 6 qvel)")
            continue
        lo, hi = model.jnt_range[j]
        axis = model.jnt_axis[j]
        dof = model.jnt_dofadr[j]
        print(f"  {jname:32s} {jtype:5s} axis={axis}  range=[{lo:+.3f}, {hi:+.3f}]"
              f"  armature={model.dof_armature[dof]:.4f}")


def print_actuators(model):
    print(f"\nACTUATORS ({model.nu}) — joint, force/torque limit (Nm)")
    for a in range(model.nu):
        joint = name(model, mujoco.mjtObj.mjOBJ_JOINT, model.actuator_trnid[a, 0])
        lo, hi = model.actuator_forcerange[a]
        print(f"  {a:2d} {joint:32s} [{lo:+.1f}, {hi:+.1f}]")


def print_sensors(model):
    print(f"\nSENSORS ({model.nsensor})")
    for s in range(model.nsensor):
        sname = name(model, mujoco.mjtObj.mjOBJ_SENSOR, s)
        stype = mujoco.mjtSensor(model.sensor_type[s]).name
        print(f"  {sname:24s} {stype:20s} dim={model.sensor_dim[s]}  noise={model.sensor_noise[s]}")


def print_cameras(model):
    print(f"\nCAMERAS ({model.ncam})")
    if model.ncam == 0:
        print("  none defined in this file")
    for c in range(model.ncam):
        body = name(model, mujoco.mjtObj.mjOBJ_BODY, model.cam_bodyid[c])
        print(f"  {name(model, mujoco.mjtObj.mjOBJ_CAMERA, c):24s} on body {body}  fovy={model.cam_fovy[c]}")


def main():
    path = Path(sys.argv[1]) if len(sys.argv) > 1 else DEFAULT_MODEL
    model = mujoco.MjModel.from_xml_path(str(path))
    print(f"Model: {path.name}")
    print(f"  nq={model.nq}  nv={model.nv}  nu={model.nu}  timestep={model.opt.timestep}s"
          f"  total mass={model.body_mass.sum():.2f} kg")
    print_bodies(model)
    print_joints(model)
    print_actuators(model)
    print_sensors(model)
    print_cameras(model)


if __name__ == "__main__":
    main()
