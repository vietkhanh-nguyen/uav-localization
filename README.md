# Indoor Localization and Navigation of an Autonomous Drone

A robust multi-sensor fusion system designed for 6-DOF UAV state estimation in GPS-denied indoor environments.

## Overview
This project tackles the challenges of indoor drone navigation, such as signal multi-path effects and dynamic disturbances, by fusing high-frequency inertial data with visual and acoustic positioning systems. The core estimation framework is built upon an Error-State Kalman Filter (ESKF).

## Key Features
* State Estimation: Error-State Kalman Filter (ESKF) for 6-DOF tracking.
* Visual-Inertial Odometry: ORB-SLAM3 integration for high-accuracy local pose estimation and multi-map handling (Atlas).
* Global Reference: Marvelmind acoustic positioning system to eliminate long-term drift.
* Simulation Environment: Distributed Software-in-the-Loop (SITL) setup using MuJoCo for accurate physics simulation and ROS2 for modular communication.

## System Architecture
The architecture employs a decentralized fusion strategy designed for deployment on hardware like a Raspberry Pi:
1. High-Frequency Prediction: Driven by raw IMU data (accelerometer and gyroscope).
2. Local Correction: ORB-SLAM3 provides relative visual-inertial updates.
3. Global Correction: Marvelmind provides absolute 3D position references via trilateration.

## Technologies Used
* ROS2
* MuJoCo
* ORB-SLAM3
* Marvelmind Indoor Navigation System
* Python / C++

## Authors
* Nguyen Viet Khanh
* Khalid Ouchaouir
* Program: Master 2 Smart Aerospace and Autonomous Systems (SAAS)
