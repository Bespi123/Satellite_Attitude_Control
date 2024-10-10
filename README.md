
# Non-Linear Control Strategies for Attitude Maneuvers in a CubeSat

This repository contains the MATLAB simulation and related resources for the paper titled **"Non-Linear Control Strategies for Attitude Maneuvers in a CubeSat with Three Reaction Wheels"**. The study, published in *International Journal of Advanced Computer Science and Applications* with DOI: 10.14569/IJACSA.2020.0111189, evaluates and compares different robust control strategies for CubeSat attitude maneuvers.

## Table of Contents
- [Introduction](#introduction)
- [Features](#features)
- [Installation](#installation)
- [Usage](#usage)
- [Control Laws](#control-laws)
- [Simulations](#simulations)
- [Results](#results)
- [License](#license)

## Introduction
This research focuses on controlling the attitude of a CubeSat subjected to various disturbances, including gravity gradient and reaction wheel misalignment. The CubeSat model is simulated using three reaction wheels, and different control strategies are implemented, namely Boskovic's robust control, Dando's adaptive control, Chen's robust control, and Schaub's quaternion feedback control.

The study compares these control laws based on several metrics:
- Steady-state error
- Error Euler Angle Integration (EULERINT)
- Average of Square of the Commanded Control Torque (ASCCT)
- Settlement time
- Computational cost

## Features
- Simulates CubeSat attitude maneuvers under different control strategies.
- Implements kinematic and dynamic equations for a CubeSat with three reaction wheels.
- Evaluates control performance under disturbances (gravity gradient, misalignment).
- Uses quaternion-based attitude representation.

## Installation
1. Clone this repository:
   ```bash
   git clone https://github.com/Bespi123/Satellite_Attitude_Control.git
   ```
2. Open MATLAB and navigate to the project directory.

## Usage
To run the simulations for regulation attitude maneuvers, execute the MATLAB script without adding the folder called Simulink into the MATLAB path:

```matlab
run('Simulacion_regulation.m')
```
The following command will run the attitude-tracking maneuvers:
```matlab
run('Simulation_tracking.m')
```
This will perform attitude control simulations using the different control laws and generate performance plots.

## Control Laws
The following control laws are implemented and compared in this study:
- **Quaternion Feedback Controller**: A basic feedback controller using quaternions.
- **Boskovic Robust Controller**: A robust control law based on the variable structure approach, accounting for input constraints.
- **Dando Adaptive Controller**: An adaptive control law that estimates the error of the inertia tensor.
- **Chen Robust Controller**: A robust finite-time control law using Fast Non-Singular Terminal Sliding Mode Surface (FNTSMS) methods.

## Simulations
The simulations include the following disturbances and configurations:
- **Gravity Gradient Torque**: The primary external disturbance affecting CubeSats in low Earth orbit.
- **Reaction Wheel Misalignment**: Disturbances introduced by slight misalignments of the reaction wheels.

### Performance Metrics:
1. **Steady-state error (ess)**: The final error between the desired and achieved orientation.
2. **Error Euler Angle Integration (EULERINT)**: Integration of the angle error about the Euler axis.
3. **Average of Square of the Commanded Control Torque (ASCCT)**: Measure of the control effort.
4. **Settlement Time (ts)**: Time taken for the system to reach a stable state within 5% of the desired orientation.

## Results
The simulation results will output performance graphs for each control law, including:
- **Euler angles** (roll, pitch, yaw)
- **Angular rates**
- **Control torque**
- **Reaction wheel rates**

The results are also compared based on the above metrics to identify the best control strategy for CubeSat attitude maneuvers.

## License
This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.