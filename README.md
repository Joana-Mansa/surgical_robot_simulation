# 🦾 Jaco2 surgical-path simulation

MATLAB coursework exploring forward/inverse kinematics and point-to-point or rectangular paths for a seven-joint Jaco2 robot in CoppeliaSim.

![Original simulation screenshot](Screenshot%202024-08-05%20140928.png)

## 📁 Explore the implementation

| Entry | Purpose |
|---|---|
| [main1.m](main1.m) | Point-to-point motion and simulator connection |
| [main2.m](main2.m) | Rectangular path and tracking plots |
| [init.m](init.m) | Robot dimensions and Denavit–Hartenberg parameters |
| [DirectKinematics.m](DirectKinematics.m), [Jacobian.m](Jacobian.m) | Pose and differential motion |
| [Homogeneous.m](Homogeneous.m), [Rot2Quat.m](Rot2Quat.m), [QuatError.m](QuatError.m) | Transform and orientation helpers |
| [trapezoidal.m](trapezoidal.m), [DrawRobot.m](DrawRobot.m) | Trajectory and visualization |

All MATLAB files are in the repository root. The three committed screenshots are historical simulation evidence.

## 🛠️ Reproduction status

The source is available, but the simulation cannot currently run end to end from this checkout. Required inputs are missing:

1. The original CoppeliaSim scene with `Revolute_joint_1` through `Revolute_joint_7`.
2. `mask_q_DH2Jaco.m`, called by both main functions to convert coordinate conventions. An identity mapping would not be a valid substitute.
3. Compatible legacy Remote API bindings: `remApi.m`, `remoteApiProto.m` and the platform library.
4. A MATLAB installation. MATLAB/Octave execution was not available during this audit.

See [the recovery and setup guide](docs/setup.md). This is a simulated trajectory experiment; it does not establish surgical accuracy or suitability for a real robot.

[Joana's project directory](https://github.com/Joana-Mansa/joana-mansa/blob/main/all-projects.md)
