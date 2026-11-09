"""Open the Booster K1 model in the MuJoCo viewer.

Run from the repository root:
    python 01_mujoco/view_k1.py

Viewer controls:
    Space             pause / resume the simulation
    Backspace         reset to the initial state
    Ctrl + mouse      drag / push the robot (right button: force, left: torque)
    Left panel        Rendering: contacts, joint axes, center of mass, etc.
    Right panel       Control: apply torque to each motor by hand
"""
from pathlib import Path

import mujoco
import mujoco.viewer

ROOT = Path(__file__).resolve().parent.parent
MODEL_PATH = ROOT / "assets/K1/K1_22dof.xml"

model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
data = mujoco.MjData(model)

print(f"Model: {MODEL_PATH.name}")
print(f"  bodies={model.nbody}  joints={model.njnt}  motors={model.nu}  sensors={model.nsensor}")
print(f"  nq={model.nq} (positions)  nv={model.nv} (velocities)  timestep={model.opt.timestep}s")
print(f"  total mass={model.body_mass.sum():.2f} kg", flush=True)

# No commands are sent to the motors, so the robot simply collapses. This is expected.
mujoco.viewer.launch(model, data)
