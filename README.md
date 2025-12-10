# Gesture-Controlled Robot Hand (ROS + Gazebo)

This project implements a simple **gesture-controlled robotic arm with a 3-finger hand** in ROS.  
Hand gestures are detected from a webcam using **OpenCV + MediaPipe**, published as ROS messages, and used to control a simulated robot arm and fingers in **Gazebo**.

## Features

- Custom URDF robot:
  - 2-DOF arm
  - Wrist joint
  - 3 fingers × 2 joints each (total 6 finger joints) :contentReference[oaicite:6]{index=6}
- Gazebo integration with `gazebo_ros_control` and position controllers for each joint. :contentReference[oaicite:7]{index=7}:contentReference[oaicite:8]{index=8}
- Real-time gesture recognition with MediaPipe:
  - OPEN_PALM
  - FIST
  - VICTORY
  - POINTING :contentReference[oaicite:9]{index=9}
- Gesture → robot mapping:
  - Different gestures move the arm and open/close individual fingers. :contentReference[oaicite:10]{index=10}

This repository is intended as a **portfolio project** showing integration of:
- Robot modeling (URDF/xacro)
- Simulation (Gazebo)
- ROS control
- Computer vision and gesture recognition.

---

## Project structure

```text
gesture-controlled-robot-hand/
├─ README.md
├─ package.xml
├─ CMakeLists.txt
├─ launch/
│  ├─ gazebo.launch          # Start Gazebo, spawn robot, load controllers
│  └─ display.launch         # Visualize robot + TF in RViz
├─ config/
│  └─ controller.yaml        # Joint position controllers (PID gains)
├─ urdf/
│  └─ my_robot.urdf.xacro    # Arm + wrist + 3-finger hand with transmissions
└─ scripts/
   ├─ gesture_recognition.py # Webcam + MediaPipe → /gesture_topic
   └─ gesture_to_robot.py    # /gesture_topic → joint position commands
