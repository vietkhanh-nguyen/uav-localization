# Skydio X2 Simulation & Control Framework

**A high-fidelity simulation and control framework for the Skydio X2 Quadcopter, built on MuJoCo physics engine with a high-performance C++ backend.**

![Language](https://img.shields.io/badge/Language-Python%20%7C%20C%2B%2B-blue)
![Physics](https://img.shields.io/badge/Physics-MuJoCo-orange)
![Status](https://img.shields.io/badge/Status-Development-green)

## 📌 Overview

This project provides a robust environment for developing and testing UAV control algorithms. It features a **hybrid architecture** where:
* **Core Dynamics & Aerodynamics** are implemented in **C++ (using Eigen)** for computational efficiency and real-time performance.
* **Simulation Loop, Control Logic & Visualization** are handled in **Python** for flexibility and rapid prototyping.
* **Physics Engine:** Uses [MuJoCo](https://mujoco.org/) for accurate rigid body dynamics and contact simulation.

The mathematical model has been **validated** against MuJoCo's internal physics engine (Ground Truth), ensuring that the custom dynamics equations (including aerodynamic drag and thrust models) are accurate.

## 🚀 Features

* **Hybrid C++/Python Core:** Seamless integration using `pybind11`.
* **Custom Aerodynamics:** Implements induced drag, thrust coefficients, and rotational drag matrices in C++.
* **Model Validation:** Tools to compare mathematical model outputs vs. MuJoCo sensor data (Acceleration, Angular Velocity).
* **Control Architectures:**
    * ✅ PID Controller (Position & Velocity tracking).
    * 🚧 *Nonlinear Control (Sliding Mode Control) - In Progress.*
* **Scenarios:** Modular scenario definitions (e.g., Circular Trajectory, Hover).

## 📂 Project Structure

```bash
my_skydio_x2/
├── controls/           # Python Control implementations (PID, etc.)
├── execution/          # Main entry points to run the simulation
├── math_model/         # C++ Source Code & Python Bindings
│   ├── drone_model.cpp # Rigid Body Dynamics (C++ / Eigen)
│   ├── aero_model.cpp  # Aerodynamic Drag & Thrust Model (C++ / Eigen)
│   └── ...
├── mjc_simulate/       # MuJoCo simulation loop and rendering wrapper
├── mjcf/               # MuJoCo XML models (drone assets, scene)
├── plots/              # Data analysis and plotting tools
├── scenario/           # Flight scenarios (Trajectory generation)
└── setup.py            # Build script for C++ extensions