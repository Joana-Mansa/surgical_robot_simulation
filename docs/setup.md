# Restore and run the simulation

## Required configuration

The code calls `simx*` methods from CoppeliaSim's **legacy Remote API** on `127.0.0.1:19997`. Current ZeroMQ bindings are not a drop-in replacement. See [CoppeliaSim's API documentation](https://manual.coppeliarobotics.com/en/remoteApiOverview.htm).

Place compatible MATLAB bindings in a local `matlab/` folder. Restore the original `mask_q_DH2Jaco.m` to the repository root, and restore the original scene. Inspect joint orientation, offsets, limits, geometry and controller assumptions before running. Main functions disable dynamics, so recorded trajectories should not be treated as dynamic robot validation.

Both entry points now resolve the project folder independently of your working directory and check missing helpers before starting.

```matlab
addpath('/path/to/surgical_robot_simulation')
[t, q, q_act] = main1; % point-to-point
% Run separately:
[t, q, q_act] = main2; % rectangular path
```

Expected figures include joint angles, position/orientation errors and Jacobian conditioning. `q` is the commanded joint trajectory; `q_act` is returned by the simulator.

## Verification performed

The complete source and its called local helpers were inspected on 2026-09-15. File links and screenshot paths were checked. No MATLAB or CoppeliaSim execution is claimed. Recovery of the coordinate helper and scene is required before numerical or motion checks can be meaningful.
