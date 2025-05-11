# 🦾 Mobile Manipulation in a Simulated Logistics Environment

Welcome to my robotics project — a fully simulated mobile manipulation system built using **CoppeliaSim** and **MATLAB**. This project was developed for an academic assignment focused on autonomous perception, navigation, and object handling within a logistics-style environment.

## 🎯 Objective

The goal of this system was to simulate a complete robotic pipeline that could:

- Navigate autonomously in a mapped environment using **SLAM**
- Detect and localize objects with **YOLOv4**
- Compute grasp trajectories using **Inverse Kinematics**
- Execute precise motions with a **UR3 robotic arm**
- Transport objects between zones with a **Pioneer 3-DX mobile base**
- Perform structured pick-and-place tasks using a **state machine controller**

---

## 🔧 System Architecture

The project is organized into modular scripts and functions. Here’s how the system is structured:

### 🗺️ SLAM & Map Building
- Teleoperated mapping via keyboard control (W/A/S/D).
- Real-time LIDAR scan acquisition using simulated Hokuyo scanner.
- Loop closures and pose graph optimization using MATLAB's `lidarSLAM`.

### 🚀 Path Planning
- A* grid-based path planner operating on the SLAM-generated map.
- PID control ensures smooth wheel velocity during navigation.
- Two main navigation tasks:
  - Move to object table (Location 1)
  - Move to object drop-off (Location 2), specific to the object label.

### 🧠 Object Detection
- YOLOv4 with CSP-DarkNet53 pre-trained model.
- Captures and processes 6 frames per run for robust detection.
- Perspective vision sensor from CoppeliaSim used for RGB input.
- Depth approximation tuned for accurate 3D transformation.

### ✋ Grasp Execution
- Cartesian grasping using custom inverse kinematics solver (`invKin8sol`).
- Grasp planning includes approach, alignment, and lift phases.
- Smooth motion via quintic trajectory interpolation.
- RG2 gripper controlled using simulated integer signals.

### 🔁 State Machine
A clean, linear state machine manages the robot’s entire workflow:
```text
INIT → MOVE_TO_LOCATION1 → DETECT_OBJECTS → GRASP_OBJECT
→ TRANSIT → MOVE_TO_LOCATION2 → DROP → RELEASE → ORIGIN → NEXT OBJECT
