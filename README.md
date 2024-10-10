# CubeSat Attitude Control Simulation: Non-Linear and Linear Model Comparison

This project simulates the attitude control of a CubeSat using both non-linear and linear models. It compares the performance of these models in terms of input torques, Euler angles, angular rates, and reaction wheel rates over time.

## Table of Contents
- [Introduction](#introduction)
- [Features](#features)
- [Installation](#installation)
- [Usage](#usage)
- [Plots and Results](#plots-and-results)
- [License](#license)

## Introduction
This MATLAB-based project aims to compare non-linear and linear control models for CubeSat attitude control. The simulation models the input torque, Euler angles, angular rates, and reaction wheel rates to assess the behavior of the CubeSat using both models.

The CubeSat control models account for:
- Reaction wheel dynamics
- Euler angles for attitude representation
- Non-linear and linear control methods

## Features
- Simulates input torques applied to the CubeSat.
- Compares the Euler angles (roll, pitch, yaw) in both linear and non-linear models.
- Simulates and compares angular rates (in RPM) for both models.
- Simulates and compares the reaction wheel rates (in RPM) for both models.
- Plots data in a 4-panel layout for clear visual comparison.

## Installation
1. Download the repository or clone it using Git:
   ```bash
   git clone https://github.com/yourusername/cubesat-simulation.git
