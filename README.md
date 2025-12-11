# Gesture-Controlled Robot Arm and Hand

This project implements a real-time gesture-controlled robotic system that combines **custom robot modeling**, **computer vision**, and **ROS-based control**.  
Both the **2-link robotic arm** and the **3-finger robotic hand** were fully designed and modeled in RobotStudio, exported as meshes, and integrated into ROS + Gazebo through URDF/Xacro.

An advanced extension attaches the custom 3‑finger hand to an **ABB GoFa industrial robot**.

---

# Requirements

- Ubuntu 20.04 LTS  
- ROS Noetic (Desktop-Full includes needed packages)  
- Gazebo 11  
- RViz  
- Python packages:  
  - `opencv-python`  
  - `mediapipe`

---

# Project Overview

The system allows you to control a simulated robot simply by showing hand gestures to your webcam.  
OpenCV + MediaPipe are used for gesture recognition, and ROS nodes translate these gestures into joint commands for the robot inside Gazebo.

This project includes:

- Custom 3D robot modeling  
- URDF/Xacro creation  
- Real-time gesture recognition  
- ROS Noetic integration  
- Gazebo simulation with ROS Control  

---

# Robot Modeling (RobotStudio → URDF)

The robot consists of:
- **2-link arm** with revolute joints  
- **A wrist joint** enabling hand rotation  
- **A fully articulated 3-finger robotic hand** (2 joints per finger)  

An advanced version has been developed:
- **ABB GoFa industrial robot**
- **The fully articulated 3-finger** robotic hand mounted on the flange

Workflow:  
RobotStudio → Mesh Export → URDF/Xacro → Gazebo → ROS Control

---

# Screenshots Showcase

## **Custom 2-Link Robot Poses Examples**

### Neutral Pose
<p align="center">
  <img src="images/2-links1.png" width="350">
</p>

<p align="center"><em>
Custom 2-link robotic arm with the articulated 3‑finger hand.
</em></p>

### Fist Gesture
<p align="center">
  <img src="images/2-links-fist.png" width="350">
</p>

<p align="center"><em>
The robot reacts to the detected "fist" gesture by closing all fingers and adjusting the wrist.
</em></p>

---

## **GoFa Extension Poses Examples**

### Open Palm
<p align="center">
  <img src="images/gofa_open_palm.png" width="350">
</p>

### Pointing
<p align="center">
  <img src="images/gofa_pointing.png" width="350">
</p>

---

## URDF Integration Visuals

<p align="center">
  <img src="images/hand_attachment_gofa.png" width="350">
</p>

<p align="center">
  <img src="images/gofa_attachment.png" width="350">
</p>

---

# Gesture Recognition (MediaPipe + OpenCV)

The ROS node `gesture_recognition.py` performs:

1. Webcam capture  
2. MediaPipe Hands landmark extraction  
3. Rule-based gesture classification  
4. Publishing gestures via `/gesture_topic`

Supported gestures:
- **OPEN_PALM**
- **FIST**
- **VICTORY**
- **POINTING**

Example detection:
<p align="center">
  <img src="images/open_palm_gesture.png" width="350">
</p>

---

# Gesture → Robot Control Architecture (ROS)

The node `gesture_to_robot.py` subscribes to `/gesture_topic` and sends joint commands for:

- Arm joints (`joint1`, `joint2`)  
- Wrist joint  
- Six finger joints  

---

# System Diagram

```
gesture_recognition.py  →  /gesture_topic  →  gesture_to_robot.py  →  Gazebo controllers
```

---

# Repository Structure

```
├── scripts/
│   ├── gesture_recognition.py
│   ├── gesture_to_robot.py
├── urdf/
│   ├── my_robot.urdf.xacro
├── config/
│   ├── controller.yaml
├── launch/
│   ├── gazebo.launch
│   ├── gesture.launch
│   ├── display.launch
├── images/
│   ├── (all screenshots and robot images)
```

---

# Code Overview

## **gesture_recognition.py — Real-Time Gesture Detection**
This script implements the complete OpenCV + MediaPipe gesture-recognition pipeline used to control the robot.
It performs real-time webcam capture, extracts MediaPipe’s 21 hand landmarks, classifies gestures using geometric rules, and publishes the recognized gesture to /gesture_topic.

Key features:
- Webcam capture optimized for low-latency (320×240, frame skipping, time-based throttling)
- MediaPipe Hands model for landmark detection (single-hand tracking)
- Custom rule-based gesture classifier:
  -OPEN_PALM
  - FIST
  - VICTORY
  - POINTING
- ROS publisher that streams recognized gestures as std_msgs/String
- On-screen visualization of landmarks and gesture labels
- Memory and performance safeguards (garbage collection, minimal CPU load)

This node is the entry point of the user-interaction pipeline: it converts human gestures into symbolic commands that the robot controller can act upon.

## **gesture_to_robot.py — Gesture → Joint Commands**
This ROS node subscribes to /gesture_topic and converts gesture messages into joint-level actuation commands for the robot arm, wrist, and all six finger joints.
Main components:
- Publishers for:
  - joint1, joint2
  - wrist_joint
  - finger{i}_joint{j} for all three fingers
- A subscriber that listens for gesture messages (String)
- Robust gesture-to-motion mappings:
  - OPEN_PALM → arm neutral, wrist rotated, all fingers open
  - FIST → fingers fully closed, characteristic curled shape
  - VICTORY → two-finger pose with customized angles
  - POINTING → index-like configuration with selected finger extension
- Automatic wait for Gazebo simulation time to become active
- Centralized logging for debugging each motion command

This script forms the bridge between perception and actuation, transforming gesture recognition into coordinated robot movement.

## **controller.yaml — Joint Controllers**
This file defines all Gazebo ROS-Control position controllers used by the robot arm, wrist, and each finger.
Every joint is assigned a JointPositionController with PID parameters tuned for stable and responsive motion in simulation.

## **gazebo.launch — Simulation Startup**
This launch file initializes the full Gazebo simulation environment and loads the custom robot into the world. It performs several key operations:
- Starts an empty Gazebo world using gazebo_ros
- Loads the URDF/Xacro robot description into the robot_description parameter
- Launches both joint_state_publisher and robot_state_publisher
- Spawns the robot model at the desired coordinates in the simulated world
- Loads all joint controllers from controller.yaml
- Registers PID gains for each joint under /gazebo_ros_control/pid_gains/
- Spawns all controllers via controller_manager
- Sends initial joint positions to place the robot in a default pose at startup

This file is the main entry point for running the robot simulation in Gazebo.

## **gesture.launch — Gesture Nodes Startup**
This launch file starts the two ROS nodes responsible for gesture-based control:

- gesture_recognition.py
  Runs the OpenCV + MediaPipe pipeline that detects hand gestures in real time and publishes them to /gesture_topic.

- gesture_listener.py
  Subscribes to the gesture topic and converts recognized gestures into robot joint commands.
  
A startup delay is included to avoid overloading ROS nodes at initialization.

This file provides the complete gesture-processing layer and can be started independently from Gazebo if needed. 

## **display.launch — URDF RViz Viewer**
This launch file loads the robot’s URDF model and opens RViz for visualization.
It performs:
- Loading the URDF/Xacro into robot_description
- Starting joint_state_publisher and robot_state_publisher
- Launching RViz to display the robot, useful for debugging link placements, joint origins, and TF frames

This file is used during robot modeling and debugging to confirm correct URDF structure before running the full simulation.

## **my_robot.urdf.xacro — Custom URDF Model**
This URDF/Xacro file defines the full robot used in the project: a 2-DOF arm, a revolute wrist, and a 3-finger articulated hand (each finger with 2 joints).
It models geometry, inertial values, joint limits, transmissions, and the Gazebo control interface.
Main components:
- Arm
  - Two modular arm links generated via the arm_link macro
  - Revolute joints: joint1 (base → link1), joint2 (link1 → link2)
  - Inertial properties
- Wrist + Hand
  -palm_link with box geometry
  - wrist_joint (link2 → palm) enabling expressive hand rotation
  - finger macro that generates:
    - link1 and link2
    - joints finger{i}_joint1 and finger{i}_joint2
    - configurable positions on the palm
- Transmissions + Gazebo
  - SimpleTransmission for every joint → enables ROS Control
  - gazebo_ros_control plugin for hardware interface integration

This URDF is the core model used by Gazebo and ROS controllers to simulate the robot’s kinematics and motion dynamics.

---

# How to Run the Simulation

### 1. Launch the robot inside Gazebo
```
roslaunch my_robot_description gazebo.launch
```

### 2. Start gesture recognition
```
rosrun gesture_control gesture_recognition.py
```

### 3. Start the gesture → robot controller
```
rosrun gesture_control gesture_to_robot.py
```

### 4. Make hand gestures in front of your webcam  
The robot will mirror them inside Gazebo.

---

# GoFa Extension

The custom hand can be mounted on an ABB GoFa robot.  
A dedicated Xacro handles flange → palm transformation.

### GoFa joint controller configuration
<p align="center">
  <img src="images/controllers_gofa.png" width="350">
</p>
<p align="center"><em>
Extended PID-based JointPositionControllers used to actuate the ABB GoFa robot joints (j1–j6).
</em></p>

### GoFa joint publishers in gesture controller
<p align="center">
  <img src="images/publishers_gofa_joints.png" width="350">
</p>
<p align="center"><em>
ROS publishers for all six GoFa joints added to the gesture controller.
</em></p>

### GoFa → custom hand URDF integration
<p align="center">
  <img src="images/gofa_attachment.png" width="350">
</p>

<p align="center">
  <img src="images/hand_attachment_gofa.png" width="350">
</p>
<p align="center"><em>
URDF/Xacro snippet showing the flange-to-palm transformation used to mount the custom hand on the ABB GoFa robot.
</em></p>

### Gesture → GoFa motion mapping
<p align="center">
  <img src="images/gesture_callback_gofa.png" width="350">
</p>
<p align="center"><em>
Gesture callback extended to drive 6-DOF GoFa arm + custom 3-finger hand.
</em></p>

---

# Gesture‑controlled Demo Videos

### Custom 2‑link arm demo  
https://github.com/user-attachments/assets/1a98822c-5cdd-445d-8f07-4513d8f4a0a6

### ABB GoFa + custom hand demo  
https://github.com/user-attachments/assets/7cfdb598-449e-4499-a33b-7a44478776d9

---

# License
MIT License

# Author
**Martina Filieri**
