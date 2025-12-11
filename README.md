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

(*All images use your original centered HTML syntax*)

## **Custom 2-Link Robot Poses**

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

## URDF Visuals

### Palm and Wrist URDF
<p align="center">
  <img src="images/hand_urdf_block1.png" width="350">
</p>

### Wrist Joint URDF
<p align="center">
  <img src="images/hand_urdf_block2.png" width="350">
</p>

---

## **GoFa Extension Poses**

### Open Palm
<p align="center">
  <img src="images/gofa_open_palm.png" width="350">
</p>

### Pointing
<p align="center">
  <img src="images/gofa_pointing.png" width="350">
</p>

---

## Gesture Recognition Code Visuals

<p align="center">
  <img src="images/gesture_recognition.png" width="350">
</p>

<p align="center">
  <img src="images/finger_tips_and_dips.png" width="350">
</p>

---

## ROS Control Visuals

<p align="center">
  <img src="images/gesture_to_robot_class_2links.png" width="350">
</p>

<p align="center">
  <img src="images/publishers_2link.png" width="350">
</p>

<p align="center">
  <img src="images/gesture_to_robot_pub.png" width="350">
</p>

<p align="center">
  <img src="images/gesture_callback_gofa.png" width="350">
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
Uses MediaPipe + OpenCV to classify hand gestures.  
Publishes results to `/gesture_topic`.

## **gesture_to_robot.py — Gesture → Joint Commands**
Maps gestures to robot poses and sends joint commands.

## **controller.yaml — Joint Controllers**
Defines all PID‑controlled joint interfaces for Gazebo.

## **gazebo.launch — Simulation Startup**
Starts Gazebo, loads the URDF, spawns the robot, loads controllers, and initializes pose.

## **gesture.launch — Gesture Nodes Startup**
Starts both gesture recognition and gesture listener nodes.

## **display.launch — URDF RViz Viewer**
Loads the robot into RViz for debugging.

## **my_robot.urdf.xacro — Custom URDF Model**
Defines:
- arm  
- wrist  
- articulated fingers  
- transmissions  
- inertial & visual geometry  
- Gazebo ROS control plugin  

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

### **GoFa integration code snippet (screenshot)**  
<p align="center">
  <img src="images/gofa_code_snippet.png" width="450">
</p>

(*Ensure `gofa_code_snippet.png` exists in the `images/` folder.*)

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
