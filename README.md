# Booster K1 — Humanoid Locomotion from Scratch

A hands-on project for learning to teach a humanoid robot to **stand, balance and walk** with
reinforcement learning, starting from an empty folder and ending on real hardware.
The target platform is the **Booster K1** humanoid by Booster Robotics.

The approach follows the pipeline described in the
[Booster Gym paper](https://arxiv.org/abs/2506.15132): train a policy in simulation with PPO,
validate it in a second simulator (sim2sim), then deploy it zero-shot on the robot (sim2real).
Each step is rebuilt here by hand so that the whole process is understood, not just run.

## The robot: Booster K1

K1 is a compact, child-sized humanoid designed for education, research and robot soccer.
It is widely used in the RoboCup Humanoid League as a common hardware platform.

| | |
|---|---|
| Height / weight | ~95 cm, ~19.5 kg (19.67 kg in the MJCF model) |
| Degrees of freedom | 22 — head 2, arms 4 × 2, legs 6 × 2 |
| Leg joints (each leg) | hip pitch / roll / yaw, knee pitch, ankle pitch / roll |
| Joint torque limits (model) | knee 112 Nm, hip pitch 68 Nm, hip roll 43 Nm, hip yaw & ankles 38.3 Nm, arms 14 Nm, head 6 Nm |
| Onboard computer | NVIDIA Jetson Orin NX (Orin AGX on some configurations) |
| Sensors | IMU, joint encoders, head-mounted RGB/stereo cameras |
| Connectivity | Wi-Fi 6, Bluetooth 5.2, Gigabit Ethernet |
| Battery life | ~30 min (2 Ah) to ~80 min (5 Ah) |
| Software | Booster Robotics SDK (DDS publisher/subscriber), low-level motor and sensor access |

Specifications vary by edition and vendor; the official reference is
[booster.tech](https://www.booster.tech/).

**What it can do** (with RL-trained policies, as demonstrated by Booster Robotics and the community):
omnidirectional walking (forward, backward, sideways, turning), recovering from pushes and
impacts, walking on slopes and uneven outdoor terrain, standing up after a fall, kicking a ball
and playing soccer, and imitating human motions such as dancing.

## Roadmap

| Stage | Topic | Runs on |
|---|---|---|
| 0 | Python environment, numpy and PyTorch basics | laptop |
| 1 | MuJoCo and the K1 model: bodies, joints, actuators, sensors, cameras | laptop |
| 2 | Standing with a hand-written PD controller (and why it is not enough) | laptop |
| 3 | RL basics: CartPole → Walker2d, a minimal PPO implementation | laptop |
| 4 | A custom K1 environment for standing and push recovery | laptop |
| 5 | Walking: booster_gym / booster_train, then a custom walking environment | rented NVIDIA GPU |
| 6 | Sim2sim: run the GPU-trained policy in MuJoCo | laptop |
| 7 | Sim2real on a physical K1 | robot |

Progress: stage 1 is in progress — the K1 model loads, its structure and sensor readings can be
printed, and the MJCF walkthrough is written. Next: adding a head camera, then the PD controller.

## Repository layout

```
assets/K1/           K1 models (MJCF + URDF) and meshes, from booster_assets (BSD-3-Clause)
01_mujoco/
  view_k1.py         open K1 in the interactive MuJoCo viewer
  inspect_model.py   print the kinematic tree, joints, actuators, sensors and cameras
  read_state.py      step the simulation and print what a policy would observe
docs/
  mjcf_overview.md   walkthrough of the K1 MJCF file
  references.md      papers, repositories and courses
```

## Quick start

Tested on macOS (Apple Silicon) with Python 3.12. MuJoCo also runs on Linux and Windows.

```bash
git clone https://github.com/Jeymsbond97/Booster.git
cd Booster
python3.12 -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt

python 01_mujoco/inspect_model.py   # model structure
python 01_mujoco/read_state.py      # sensor readings while the robot falls
python 01_mujoco/view_k1.py         # interactive viewer
```

Without control the robot collapses in the viewer — that is expected; teaching it not to is the
point of this project.

## Model files

| File | Description |
|---|---|
| `K1_22dof.xml` | MuJoCo MJCF, ankles modelled as serial pitch/roll joints |
| `K1_22dof_parallel.xml` | MJCF with the real parallel ankle linkage |
| `K1_22dof.urdf` | URDF for Isaac Gym / Isaac Lab |
| `K1_locomotion.urdf` | URDF with head and arms fixed, for locomotion training |

## Credits and license

- Robot models and meshes in `assets/K1/` are from
  [BoosterRobotics/booster_assets](https://github.com/BoosterRobotics/booster_assets)
  (commit `12a9751`), licensed under BSD-3-Clause — see `assets/K1/LICENSE`.
- Method based on *Booster Gym: An End-to-End Reinforcement Learning Framework for Humanoid
  Robot Locomotion* (Wang et al., 2025).
- Everything else in this repository is released under the MIT License — see `LICENSE`.

This is an independent learning project and is not affiliated with Booster Robotics.
