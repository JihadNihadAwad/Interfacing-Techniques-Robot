# Interfacing Techniques Robot - Maze Solving Robot

This project contains the **main merged code** for a maze-solving robot developed as part of the Interfacing Techniques course. The robot autonomously explores and solves a maze using a combination of custom hardware interfacing, algorithmic pathfinding (Flood Fill), and sensor fusion for robust navigation.

## Overview

The robot is built on a microcontroller platform and uses a combination of **Lidar (ToF) sensors**, **MPU6050 IMU**, and **motor encoders** to perceive its environment and control precise movement. 
The core algorithm for maze solving is a customized implementation of the **Flood Fill** technique, which was adapted and further developed from our MMS algorithms 
[Interfacing-Techniques-MMS](https://github.com/JihadNihadAwad/Interfacing-Techniques-MMS).

## Key Features and Custom Solutions

### 1. **Sensor Fusion for Navigation**

- **Lidar Sensors:** Three VL53L0X ToF Lidar sensors are mounted at the front and on both sides of the robot to detect walls and distances in the maze.
- **MPU6050 IMU:** Used to track yaw (rotation angle) for precise turning and orientation, including gyro calibration and drift correction.
- **Motor Encoders:** Quadrature encoders on both drive motors provide feedback for distance measurement and assist with straight-line accuracy.

Custom logic fuses IMU and encoder data for more reliable rotation and straight-line movement, compensating for slippage or sensor noise.

### 2. **Custom Flood Fill Maze Algorithm**

- The robot dynamically builds a map of the maze as it explores, using real-time wall detection.
- Walls are logged and shared between adjacent cells for consistency.
- The Flood Fill algorithm calculates distances to goal cells, allowing the robot to always head towards the shortest available path.
- The code supports **multiple goal cells** (center of the maze) and recalculates the flood map on-the-fly as new walls are discovered.

### 3. **Precision Motor Control and PID Tuning**

- Custom PID controllers allow for both straight movement and turning, with separate tuning for each mode.
- Speed is dynamically adjusted based on wall-following feedback, using Lidar readings to maintain centering between walls.
- The robot detects and handles edge cases like dead-ends and intersections, performing turns as needed.

### 4. **Hardware Initialization and Recovery**

- Robust initialization routines for all sensors, with error handling for failed communication.
- Continuous range measurements with automatic restart for each Lidar sensor.
- Custom recovery and calibration routines for the IMU to ensure consistent heading.

## Maze Solving Flow

1. **Initialization:** All sensors and motors are initialized, and the robot is placed at the maze start.
2. **Exploration:** The robot moves cell by cell, detecting walls, logging them, and updating its internal map.
3. **Pathfinding:** At each step, the robot uses its flood-filled map to choose the direction with the lowest distance to the goal.
4. **Wall-Following and Correction:** If the robot deviates from the center or approaches a wall, PID and wall-following logic make real-time adjustments.
5. **Goal Detection:** Upon reaching the maze center, the robot stops and cleans up.

## Hardware Used

- 3x VL53L0X Lidar (ToF) Sensors
- 1x MPU6050 IMU
- 2x DC Motors with Quadrature Encoders
- ESP32 (or compatible microcontroller)
- Custom Chassis and Power Management

---

**Project by:** [JihadNihadAwad](https://github.com/JihadNihadAwad)
