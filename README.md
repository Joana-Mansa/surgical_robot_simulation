# surgical_robot_simulation
A MATLAB project synced with the CoppeliaSim software simulating a surgical robot making incisions on the chest of a patient.
# Surgical Robot Simulation

![MATLAB](https://img.shields.io/badge/MATLAB-R2022b+-blue.svg)
![CoppeliaSim](https://img.shields.io/badge/CoppeliaSim-4.2.0+-green.svg)
![License](https://img.shields.io/badge/License-MIT-yellow.svg)

A MATLAB project synchronizing with CoppeliaSim software to simulate a surgical robot making precise incisions on a patient's chest. This project implements direct kinematics, inverse kinematics, and motion control of a 7-DOF Jaco2 robotic arm.

<!-- ## Table of Contents
- [Project Overview](#project-overview)
- [Features](#features)
- [Repository Contents](#repository-contents)
- [Requirements](#requirements)
- [Installation](#installation)
- [Usage](#usage)
- [Simulation Modes](#simulation-modes)
- [Implementation Details](#implementation-details)
- [Results and Visualization](#results-and-visualization)
- [License](#license)
- [Contact](#contact) -->

![Robot Simulation](https://raw.githubusercontent.com/Joana-Mansa/surgical_robot_simulation/main/Screenshot%202024-08-05%20140928.png)

## Project Overview

This simulation project demonstrates the control of a surgical robot performing precise movements for surgical procedures. The implementation includes:

- Forward and inverse kinematics for a 7-DOF robotic arm (Jaco2)
- Trajectory planning with trapezoidal velocity profiles
- Real-time communication between MATLAB and CoppeliaSim
- Motion control with position and orientation error minimization
- Visualization of robot movement and performance metrics

## Features

- **Direct Kinematics**: Calculate end-effector position and orientation from joint angles
- **Inverse Kinematics**: Determine joint angles for desired end-effector position and orientation
- **Trajectory Planning**: Generate smooth paths with trapezoidal velocity profiles
- **Visualization**: Plot joint positions, velocities, errors, and 3D robot movements
- **Real-time Simulation**: Interface with CoppeliaSim for realistic robotic arm visualization
- **Rectangular Path Motion**: Execute precise rectangular incision patterns
- **Performance Analysis**: Monitor Jacobian condition number and tracking errors

## Repository Contents
```
surgical_robot_simulation/
│
├── main1.m                  # Point-to-point motion control implementation
├── main2.m                  # Rectangular path following for surgical incisions
├── init.m                   # Initialization of robot parameters and DH configuration
│
├── Kinematics/
│   ├── DirectKinematics.m   # Forward kinematics using DH parameters
│   ├── Jacobian.m           # Calculation of the robot's Jacobian matrix
│   ├── Homogeneous.m        # Homogeneous transformation matrices
│   ├── Rot2Quat.m           # Rotation matrix to quaternion conversion
│   ├── QuatError.m          # Quaternion error calculation
│   └── CheckVector.m        # Vector validation utility
│
├── Visualization/
│   └── DrawRobot.m          # Visualization of the robot arm in MATLAB
│
├── Motion/
│   └── trapezoidal.m        # Trapezoidal velocity profile generator
│
└── Screenshots/
    ├── Screenshot 2024-08-05 140244.png
    ├── Screenshot 2024-08-05 140431.png
    └── Screenshot 2024-08-05 140928.png
```
## Requirements

- MATLAB (tested with R2022b or newer)
- CoppeliaSim (formerly V-REP) 4.2.0 or newer
- MATLAB Remote API for CoppeliaSim

## Installation

1. Clone this repository:
   ```
   git clone https://github.com/Joana-Mansa/surgical_robot_simulation.git
   ```

2. Install CoppeliaSim from [Coppelia Robotics website](https://www.coppeliarobotics.com/downloads)

3. Set up the MATLAB-CoppeliaSim communication:
   - Copy the remote API files from your CoppeliaSim installation to the `matlab` folder in the project directory
   - These files include `remApi.m`, `remoteApi.dll` (Windows), `remoteApi.dylib` (macOS), or `remoteApi.so` (Linux)

## Usage

### Simulation Setup

1. Start CoppeliaSim and open the Jaco2 robot scene (not included in the repository, use a standard Jaco2 model)
2. Ensure the CoppeliaSim Remote API server is enabled (default port: 19997)
3. Run one of the main MATLAB scripts

### Running Simulations

For point-to-point movement:
```matlab
[time, joint_positions, actual_positions] = main1;
```

For rectangular path following (simulating surgical incisions):
```matlab
[time, joint_positions, actual_positions] = main2;
```

## Simulation Modes

### Point-to-Point Movement (main1.m)
Moves the robotic arm from an initial position to a target position using a smooth trajectory.

### Rectangular Path Following (main2.m)
Controls the robot to follow a rectangular path, similar to making surgical incisions in a pattern:
1. Moves to the first corner of the rectangle
2. Follows the edges of the rectangle with stops at each corner
3. Returns to the initial position
4. Generates plots showing the robot's performance

## Implementation Details

### Kinematics
- Uses Denavit-Hartenberg parameters for robot modeling
- Implements forward kinematics to determine end-effector pose
- Applies inverse kinematics using the Jacobian pseudoinverse method

### Motion Control
- Implements trapezoidal velocity profiles for smooth acceleration and deceleration
- Handles both position and orientation (quaternion) control
- Monitors robot performance via Jacobian condition number

### CoppeliaSim Integration
- Establishes communication using the Remote API
- Synchronizes simulation time steps between MATLAB and CoppeliaSim
- Transfers joint angles to the simulated robot in real-time

## Results and Visualization

The simulation generates several visualizations to help understand the robot's performance:

- Joint positions and velocities over time
- Position and orientation errors
- Jacobian condition number analysis
- 3D visualization of the robot's movement
- Path tracing showing the end-effector trajectory

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Contact

Please get in touch with the repository owner [Joana-Mansa](https://github.com/Joana-Mansa).
