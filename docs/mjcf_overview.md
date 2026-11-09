# K1 MJCF walkthrough

File: `assets/K1/K1_22dof.xml` (217 lines). Print the full structure with
`python 01_mujoco/inspect_model.py`.

## Top-level sections

```
<mujoco>
  <compiler>    angles in radians, mesh directory
  <option>      physics: timestep 0.001 s (1000 Hz), Newton solver
  <default>     default contact softness for all geoms
  <asset>       22 STL meshes (visual shape) and the floor texture
  <worldbody>   the floor and the robot's body tree
  <actuator>    22 torque motors with their limits
  <sensor>      IMU orientation and gyro
</mujoco>
```

## Bodies

`trunk` (6.5 kg) is the root. Five branches hang from it: head, left arm, right arm, left leg,
right leg. Each `<body>` has a `pos` relative to its parent and an `<inertial>` block (mass,
center of mass, inertia) — this is what the physics uses, not the mesh.

The heaviest parts of each leg are the hip-yaw link (1.65 kg) and the shank (1.5 kg).
`head_realsense_rgb_link` and `head_booster_stereo_rgb_link` are empty bodies that only mark
where the real cameras are mounted.

## Joints

- `world_joint` is a `free` joint: it lets the torso move freely in the world.
  It takes 7 position values (xyz + quaternion) and 6 velocity values.
- The other 22 are `hinge` joints, each rotating around one axis
  (`0 1 0` = pitch, `1 0 0` = roll, `0 0 1` = yaw) within a `range` in radians.
- Hence `nq = 7 + 22 = 29` and `nv = 6 + 22 = 28`.
- `armature` adds the reflected inertia of the motor rotor and gearbox. It is much larger on the
  legs (0.03–0.1) than on the arms (0.001), because leg motors use higher gear ratios.
- Knees only bend one way: range `[0, 2.321]`.

## Visual vs. collision geometry

Every link has a mesh geom with `contype="0" conaffinity="0"` — it is drawn but never collides.
Collisions use simple primitives instead: a box for the torso, cylinders for hips and knees,
and an 18 × 7 × 3.6 cm box under each foot. Simple shapes make contacts faster and more stable.

The floor has friction `0.4`, which is fairly low; locomotion training usually randomizes it.

## Actuators

`<motor>` elements apply torque directly to a joint, clipped to `forcerange`:

| Joint group | Limit (Nm) |
|---|---|
| Knee pitch | 112 |
| Hip pitch | 68 |
| Hip roll | 43 |
| Hip yaw, ankle pitch, ankle roll | 38.3 |
| Arms | 14 |
| Head | 6 |

There is no position control in the file. A PD controller (`τ = kp·(q_des − q) − kd·q̇`) has to
be written in code — that is stage 2.

## Sensors

Only two, both on the `imu` site at the torso origin:

- `orientation` — `framequat`, a quaternion (w, x, y, z), noise 0.001
- `angular-velocity` — `gyro`, rad/s, noise 0.005

Joint positions and velocities have no explicit sensors; they are read from `data.qpos[7:]` and
`data.qvel[6:]`, which matches the encoders on the real robot.

## Cameras

None are defined (`ncam = 0`). To render from the robot's point of view, a `<camera>` element
can be added inside one of the two head camera bodies.
