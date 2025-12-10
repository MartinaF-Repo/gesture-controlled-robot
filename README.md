# Gesture-Controlled Robot Arm and Hand
This project implements a real-time gesture-controlled robotic system that combines **custom robot modeling**, **computer vision**, and **ROS-based control**.  
Both the **2-link robotic arm** and the **3-finger robotic hand** were **fully designed and modeled by me in RobotStudio**, exported as meshes, and integrated into ROS + Gazebo through custom URDFs.

An advanced extension of the project attaches the same custom finger hand to an **ABB GoFa industrial robot**.

---

# Project Overview

The system allows you to control a robot arm and its fingers by simply showing hand gestures to a webcam.  
Gesture recognition is performed using **OpenCV + MediaPipe**, and gestures are mapped to ROS topics that control each robot joint in the Gazebo simulation.

This project involved:
- 3D robot modeling (RobotStudio)
- URDF creation
- ROS Noetic / Gazebo integration
- Real-time computer vision
- Simulation workflow for robotics

---

# Custom Robot Modeling (Created in RobotStudio)

Both the **2-link arm** and the **3-finger robotic hand** were **designed from scratch in RobotStudio**, including:

### Mechanical structure  
- Joint layout  
- Link proportions  
- Wrist design  
- Three fully articulated fingers (2 joints each)

### Exported 3D assets  
The models were exported as meshes for integration into the URDF robot description.

### Kinematic design  
- Defining rotational axes  
- Parent-child link structure  

### Integration pipeline  
RobotStudio → Mesh export → URDF/XACRO → Gazebo → ROS Control

---

# Screenshots Showcase

## **Custom 2-Link Robot Poses**
### Neutral Pose
<p align="center">
  <img src="docs/images/2-links1.png" width="350">
</p>

<p align="center"><em>
Custom 2-link robotic arm with my self-modelled 3-finger hand, imported into Gazebo via URDF.
</em></p>

### Fist gesture
<p align="center">
  <img src="docs/images/2-links-fist.png" width="350">
</p>

<p align="center"><em>
The robot reacts to the detected "fist" gesture by closing all fingers and adjusting the wrist.
</em></p>

## URDF

### Palm and wrist URDF
<p align="center">
  <img src="docs/images/hand_urdf_block1.png" width="350">
</p>

<p align="center"><em>
URDF definitions for the palm link, including inertial values, visual geometry, and collision shapes.
</em></p>

### Wrist joint URDF
<p align="center">
  <img src="docs/images/hand_urdf_block2.png" width="350">
</p>

<p align="center"><em>
Wrist joint definition connecting the hand assembly to the arm, with joint limits and control properties.
</em></p>


## **GoFa Extension Poses**

### Open Palm
<p align="center">
  <img src="docs/images/gofa_open_palm.png" width="350">
</p>

<p align="center"><em>
The custom finger hand attached to the ABB GoFa robot, responding to the “open palm” gesture.
</em></p>

### Pointing
<p align="center">
  <img src="docs/images/gofa_pointing.png" width="350">
</p>

<p align="center"><em>
Gesture-driven “pointing” motion using the GoFa's 6-DOF arm and the articulated custom finger hand.
</em></p>

## **Gesture Recognition Code**

<p align="center">
  <img src="docs/images/docs/images/gesture_recognition.png" width="350">
</p>

<p align="center"><em>
This snippet implements the core gesture classification algorithm.
After extracting MediaPipe hand landmarks, the code determines whether each finger is open or closed by comparing fingertip coordinates to DIP joint coordinates. Based on the resulting binary pattern, it classifies gestures such as OPEN_PALM, FIST, VICTORY, and POINTING.
This logic enables real-time interpretation of human hand poses.
</em></p>

<p align="center">
  <img src="docs/images/docs/images/finger_tips_and_dips.png" width="350">
</p>

<p align="center"><em>
This snippet initializes the MediaPipe Hands model and specifies the landmark indices used for finger-state detection.
FINGER_TIPS and FINGER_DIP store the exact MediaPipe landmark IDs for each finger, enabling the system to evaluate finger openness by comparing their positions. The camera is also configured for real-time processing using OpenCV.
</em></p>

## **ROS Control**

### Gesture-to-Robot controller class
<p align="center">
  <img src="docs/images/gesture_to_robot_class_2links.png" width="350">
</p>

<p align="center"><em>
Initialization of the ROS control node, publishers, and gesture subscriber for real-time robot actuation.
</em></p>


### ROS publisher dictionary
<p align="center">
  <img src="docs/images/publishers_2link.png" width="350">
</p>

<p align="center"><em>
Dictionary-based joint publisher mapping, enabling dynamic routing of gesture commands to each finger joint.
</em></p>

### Gesture callback logic
<p align="center">
  <img src="docs/images/gesture_callback_gofa.png" width="350">
</p>

<p align="center"><em>
Gesture mapping logic that sets joint targets based on recognized hand poses such as open palm, fist, or pointing.
</em></p>


## **URDF Integration**

### GoFa hand mounting
<p align="center">
  <img src="docs/images/hand_attachment_gofa.png" width="350">
</p>

<p align="center"><em>
URDF/Xacro showing the hand attachment to the GoFa robot via a custom wrist joint and link transform.
</em></p>

<p align="center">
  <img src="docs/images/gofa_attachment.png" width="350">
</p>

<p align="center"><em>
Part of the URDF that defines the parent–child relationship between the GoFa flange and the custom palm link.
</em></p>


---

# 🧠 Gesture Recognition Pipeline (MediaPipe + OpenCV)

The gesture recognition node performs:

### 1. Hand Landmark Extraction  
MediaPipe Hands detects 21 landmarks on the user’s hand in real-time.

### 2. Gesture Classification  
A set of geometric rules uses:
- relative position of fingertips  
- DIP and PIP joint angles  
- finger openness/closedness  
to classify gestures such as:
- **OPEN_PALM**
- **FIST**
- **VICTORY**
- **POINTING**

### 3. ROS Publishing  
The recognized gesture is published to:

```
/gesture_topic         (std_msgs/String)
```

### 4. Visual Feedback  
OpenCV overlays detected gesture and hand landmarks on the webcam feed.

---

# 🤖 Gesture → Robot Control Architecture (ROS)

A dedicated ROS node (`gesture_to_robot.py`) interprets gestures into joint commands.

### Core Responsibilities:
- Subscribe to `/gesture_topic`
- Publish joint commands using `std_msgs/Float64`
- Maintain per-joint routing via a publisher dictionary
- Execute mapped pose behaviors:
  - OPEN_PALM → open all fingers
  - FIST → close all fingers
  - VICTORY → two-finger-mode
  - POINTING → index-like finger extension

### ROS Nodes Overview

```
+---------------------------+
| gesture_recognition.py    |
| (OpenCV + MediaPipe)      |
+-------------+-------------+
              |
              v
    publishes /gesture_topic
              |
+-------------+-------------+
| gesture_to_robot.py       |
| Sends joint commands      |
+-------------+-------------+
              |
              v
+---------------------------+
| Gazebo Controllers        |
| (arm + wrist + fingers)   |
+---------------------------+
```

---

# 🦾 URDF & Simulation Design

### ✔ 2-Link Arm  
- Base → Link1 → Link2  
- Joint axis definitions  
- Inertial parameters  
- Transmission blocks for Gazebo control  

### ✔ 3-Finger Hand  
Each finger includes:
- 2 rotational joints  
- Proper origins and axes  
- Mesh alignment  
- Realistic articulation  

### ✔ Wrist Joint  
Connects the arm and hand, enabling expressive pose control.

### ✔ Gazebo Integration  
- `gazebo_ros_control` plugin  
- Position controllers for each joint  
- Realistic movement with PID parameters

---

# Advanced Extension: ABB GoFa Integration

As an advanced demonstration, the custom 3‑finger hand was attached to an ABB GoFa industrial robot.

---

## Gesture-controlled Demo Video

### Custom 2-link arm
https://github.com/user-attachments/assets/1a98822c-5cdd-445d-8f07-4513d8f4a0a6

### ABB GoFa + custom hand
https://github.com/user-attachments/assets/7cfdb598-449e-4499-a33b-7a44478776d9

# License
MIT License

---

# Author
**Martina Filieri**  
AI Engineer

---
