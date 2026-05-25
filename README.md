# Digital Twin of 6-DOF Robotic Manipulator – PID vs MPC Control

## Overview
This project implements a Digital Twin of a 6-DOF Puma560 robotic manipulator using MATLAB, Simulink, Simscape Multibody, Robotics Toolbox, and MPC Toolbox.

The project compares PID and Model Predictive Control (MPC) for robotic trajectory tracking under multiple robustness scenarios.

---

## Tools Used
- MATLAB R2026a
- Simulink
- Simscape Multibody
- Robotics Toolbox
- MPC Toolbox

---

## Key Features
- 6-DOF rigid body dynamics
- Cubic polynomial trajectory planning
- PID controller with gravity compensation
- Linearized MPC controller
- Jacobian analysis
- Robustness testing

---

## Validation Scenarios
1. Ideal Conditions
2. Sensor Noise
3. External Disturbances
4. Mass Uncertainty

---

## Results
- MPC achieved 8x lower tracking error than PID
- 21x better robustness under sensor noise
- Stable tracking under disturbances and uncertainty

---

## Included Files
- MATLAB source code
- Simulink block diagrams
- Result plots
- Control comparison figures
- Robustness analysis

---

## Author
Yogesh Umesh  
M.Sc. Mechatronics & Robotics  
Hochschule Schmalkalden, Germany
