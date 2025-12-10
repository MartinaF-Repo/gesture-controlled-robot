# Gesture-Controlled Robot Arm and Hand (ROS + Gazebo + OpenCV + MediaPipe)

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
- Ensuring stable IK/FK behavior in Gazebo

### Integration pipeline  
RobotStudio → Mesh export → URDF/XACRO → Gazebo → ROS Control

---

# Screenshots Showcase

### **Custom 2-Link Robot**

<img src="docs/images/2-links1.png" width="400">
<img src="docs/images/2-links-fist.png" width="400">

### **GoFa Extension**

![GoFa Open Palm](docs/images/gofa_open_palm.png)
![GoFa Pointing](docs/images/gofa_pointing.png)


### **Gesture Recognition**

![Gesture Recognition](docs/images/gesture_recognition.png)
![Finger Logic](docs/images/finger_tips_and_dips.png)


### **ROS Control**

![Controller Class](docs/images/gesture_to_robot_class_2links.png)
![Publishers](docs/images/publishers_2link.png)
![Gesture Callback](docs/images/gesture_callback_gofa.png)


### **URDF Integration**

![Hand Attachment](docs/images/hand_attachment_gofa.png)
![GoFa Attach](docs/images/gofa_attachment.png)


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
