<div align="center">

# UR5e Kinect Vision Pick-and-Place

### Voice-Commanded Robot Manipulation with 3-D Vision

[![ROS 2 Humble](https://img.shields.io/badge/ROS%202-Humble-blue?logo=ros&logoColor=white)](https://docs.ros.org/en/humble/)
[![Python 3.10](https://img.shields.io/badge/Python-3.10-3776AB?logo=python&logoColor=white)](https://www.python.org/)
[![MoveIt 2](https://img.shields.io/badge/MoveIt-2-orange?logo=ros&logoColor=white)](https://moveit.ros.org/)
[![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)
[![Ubuntu 22.04](https://img.shields.io/badge/Ubuntu-22.04-E95420?logo=ubuntu&logoColor=white)](https://ubuntu.com/)
[![OpenCV](https://img.shields.io/badge/OpenCV-4.x-5C3EE8?logo=opencv&logoColor=white)](https://opencv.org/)

A fully integrated ROS 2 robotics system that fuses **depth-camera vision**, **real-time shape detection**, and **voice-activated commands** to autonomously pick and place coloured objects with a **Universal Robots UR5e** manipulator arm.

</div>

---

## Table of Contents

- [Overview](#overview)
- [System Architecture](#system-architecture)
- [Features](#features)
- [Project Structure](#project-structure)
- [Prerequisites](#prerequisites)
- [Installation](#installation)
- [Quick Start — Simulation](#quick-start--simulation)
- [Real Hardware Usage](#real-hardware-usage)
- [ROS 2 Topics & Parameters](#ros-2-topics--parameters)
- [Node Reference](#node-reference)
- [Configuration](#configuration)
- [Team](#team)
- [License](#license)

---

## Overview

This project implements a complete **voice-commanded pick-and-place pipeline** for the **UR5e** collaborative robot arm. A Microsoft Kinect 360 depth camera continuously scans the workspace; when a voice command is issued (e.g. *"circle one"* or *"square two"*), the system:

1. Identifies the target object using HSV colour segmentation and contour analysis
2. Back-projects its pixel coordinates to a 3-D world-frame pose using the Kinect depth channel
3. Plans a full grasp trajectory via **MoveIt 2** — pre-grasp, grasp, lift, transport, drop
4. Executes the trajectory on the UR5e through the `scaled_joint_trajectory_controller`
5. Controls the **Robotiq 2F-85 gripper** to open / close around the object

A fully-featured **simulation mode** (no hardware required) reproduces the entire pipeline using a synthetic Kinect stream and fake UR5e hardware, making it possible to develop and demo on any ROS 2 Humble machine.

---

## System Architecture

```
┌─────────────────────────────────────────────────────────────────────┐
│                        SENSOR LAYER                                 │
│                                                                     │
│   Microsoft Kinect 360               Voice Microphone               │
│   (or Mock Kinect in sim)            (SpeechRecognition)            │
│          │                                    │                     │
│     /kinect/rgb                        /voice_command               │
│     /kinect/depth                      (std_msgs/String)            │
└─────────────┬──────────────────────────────────┬────────────────────┘
              │                                  │
              ▼                                  ▼
┌─────────────────────────┐       ┌──────────────────────────────────┐
│     detection_node      │       │        voice_command_node        │
│  HSV colour segmentation│       │  Google Speech API → shape_N     │
│  Contour shape analysis │       └──────────────┬───────────────────┘
│  /detection/pixel ──────┼──────────────────────┤                    
└─────────────────────────┘                      │                    
              │                                  │                    
              ▼                                  │                    
┌─────────────────────────┐                      │                    
│  depth_extractor_node   │                      │                    
│  Median-patch depth read│                      │                    
│  /detection/pixel_with_ │                      │                    
│  depth                  │                      │                    
└──────────────┬──────────┘                      │                    
               │                                 │                    
               ▼                                 │                    
┌─────────────────────────┐                      │                    
│  coord_transform_node   │                      │                    
│  Pixel + depth → 3-D    │                      │                    
│  Camera-frame XYZ       │                      │                    
│  /detection/coords      │                      │                    
└──────────────┬──────────┘                      │                    
               │                                 │                    
               └──────────► detection_node ◄─────┘                    
                           (target matching)                          
                                  │                                   
                     /detection/target_coords                         
                                  │                                   
                                  ▼                                   
              ┌───────────────────────────────────┐                   
              │       robot_commander_node        │                   
              │  Camera → robot-base transform    │                   
              │  MoveIt 2 Cartesian planning      │                   
              │  9-step pick-and-place sequence   │                   
              └──────────────┬────────────────────┘                   
                             │                                        
              ┌──────────────┴────────────────────┐                   
              ▼                                   ▼                   
┌─────────────────────────┐       ┌───────────────────────────────┐   
│   /move_group (MoveIt2) │       │  /scaled_joint_trajectory_    │   
│   Cartesian moves       │       │  controller/joint_trajectory  │   
│   IK + collision check  │       │  Joint-space moves            │   
└─────────────────────────┘       └───────────────────────────────┘   
              │                                   │                   
              └──────────────┬────────────────────┘                   
                             ▼                                        
              ┌─────────────────────────────────┐                     
              │      UR5e Manipulator Arm       │                     
              │   (real or fake hardware)       │                     
              │   + Robotiq 2F-85 Gripper       │                     
              └─────────────────────────────────┘                     
```

---

## Features

| Feature | Details |
|---|---|
| **Voice control** | Google Speech Recognition — say *"circle one"*, *"square two"*, *"triangle three"* |
| **Real-time vision** | HSV colour segmentation + contour analysis at 10–15 Hz |
| **Depth fusion** | Median-patch depth extraction from Kinect IR channel |
| **3-D localisation** | Pinhole back-projection with optional calibration file |
| **Motion planning** | MoveIt 2 / OMPL Cartesian planner with configurable IK tolerance |
| **9-step sequence** | Home → Open gripper → Pre-grasp → Grasp → Close → Lift → Drop zone → Open → Home |
| **Simulation mode** | Zero hardware needed — synthetic Kinect stream + UR5e fake hardware |
| **Safety recovery** | Automatic home recovery on planning or execution failure |
| **Configurable** | All offsets, tolerances, gripper positions, and timings via ROS 2 parameters |

---

## Project Structure

```
ur5e-kinect-pick-and-place/
├── src/
│   └── kinect_picking/             # ROS 2 ament_python package
│       ├── kinect_picking/
│       │   ├── kinect_node.py          # Real Kinect 360 driver (libfreenect)
│       │   ├── mock_kinect_node.py     # Synthetic stream for simulation
│       │   ├── detection_node.py       # HSV colour + shape detector
│       │   ├── depth_extractor_node.py # Kinect depth reader
│       │   ├── coord_transform_node.py # Pixel → 3-D transform
│       │   ├── robot_commander_node.py # MoveIt 2 pick-and-place executor
│       │   └── voice_command_node.py   # Google Speech → ROS topic
│       ├── launch/
│       │   ├── ur_fake.launch.py           # Simulation: UR5e fake hw + MoveIt 2
│       │   ├── kinect_picking_sim.launch.py# Simulation: full vision pipeline
│       │   └── kinect_picking.launch.py    # Real hardware: full pipeline
│       ├── test/
│       ├── package.xml
│       ├── setup.py
│       └── setup.cfg
├── run_simulation.sh       # One-command simulation launcher
├── run_full_system.sh      # One-command real-hardware launcher
├── LICENSE
└── README.md
```

---

## Prerequisites

### System

| Requirement | Version |
|---|---|
| Ubuntu | 22.04 LTS |
| ROS 2 | Humble Hawksbill |
| Python | 3.10 |
| MoveIt 2 | Humble |

### ROS 2 Packages

```bash
sudo apt install -y \
  ros-humble-ur \
  ros-humble-ur-robot-driver \
  ros-humble-ur-moveit-config \
  ros-humble-moveit \
  ros-humble-cv-bridge \
  ros-humble-vision-opencv
```

### Python Dependencies

```bash
pip install \
  opencv-python \
  numpy \
  SpeechRecognition \
  pyaudio
```

### Real Hardware Only

```bash
# libfreenect for Kinect 360
sudo apt install -y freenect python3-freenect

# Robotiq gripper adapter (from your UR driver workspace)
# Source ur_driver workspace before running
```

> **Note on Python versions:** ROS 2 Humble requires Python 3.10.  
> If your system default `python3` is newer (3.11+), use `python3.10` explicitly.  
> All scripts in this repo already handle this automatically.

---

## Installation

### 1. Clone the repository

```bash
git clone https://github.com/tarekokasha22/ur5e-kinect-pick-and-place.git
cd ur5e-kinect-pick-and-place
```

### 2. Source ROS 2 Humble

```bash
source /opt/ros/humble/setup.bash
```

### 3. Build the package

```bash
cd src   # workspace root is one level up from src/
# build from workspace root
cd ..
python3.10 -m colcon build --symlink-install
source install/setup.bash
```

> Alternatively use the provided launcher scripts — they build automatically.

---

## Quick Start — Simulation

No Kinect or UR5e robot required. The simulation uses a synthetic RGB+depth stream and fake UR5e hardware with full MoveIt 2 motion planning.

### One-command launch

```bash
cd ur5e-kinect-pick-and-place
bash run_simulation.sh
```

This automatically:
- Builds the workspace
- Opens **Terminal 1** → UR5e fake hardware + MoveIt 2 + RViz
- Opens **Terminal 2** → Vision pipeline (mock Kinect, detection, commander)

RViz appears in ~15 seconds. The detection window opens showing three synthetic shapes.

### Send pick commands

After both terminals are running, open a third terminal:

```bash
source install/setup.bash

# Pick the pink circle
python3.10 /opt/ros/humble/bin/ros2 topic pub --once /voice_command \
  std_msgs/msg/String "data: 'circle_1'"

# Pick the green square
python3.10 /opt/ros/humble/bin/ros2 topic pub --once /voice_command \
  std_msgs/msg/String "data: 'square_1'"

# Pick the pink triangle
python3.10 /opt/ros/humble/bin/ros2 topic pub --once /voice_command \
  std_msgs/msg/String "data: 'triangle_1'"
```

The detection window highlights the target in **cyan**, and the UR5e arm in RViz executes the full 9-step pick-and-place sequence.

### Simulation scene

The mock Kinect publishes three synthetic coloured shapes at a fixed depth of **0.65 m**:

| Object | Colour | Pixel centre | Robot-frame approx. |
|---|---|---|---|
| Circle 1 | Pink | (200, 200) | X=0.35, Y=0.15, Z=0.45 m |
| Square 1 | Green | (420, 280) | X=0.66, Y=0.26, Z=0.45 m |
| Triangle 1 | Pink | (300, 360) | X=0.47, Y=0.30, Z=0.45 m |

---

## Real Hardware Usage

### Prerequisites

1. UR5e powered on and connected via Ethernet (default IP: `192.168.1.102`)
2. Kinect 360 connected via USB
3. Robotiq 2F-85 gripper mounted and `ur_driver` workspace built and sourced

### Launch

```bash
# Default robot IP
bash run_full_system.sh

# Custom robot IP
bash run_full_system.sh 192.168.0.10
```

This opens three coordinated terminals:

| Terminal | Contents |
|---|---|
| 1 | UR5e robot driver (`ur_control.launch.py`) |
| 2 | MoveIt 2 motion planning + RViz |
| 3 | Kinect vision pipeline + voice command node + robot commander |

### Voice commands

Speak naturally into your microphone:

```
"circle one"    → picks Circle_1
"square two"    → picks Square_2
"triangle one"  → picks Triangle_1
```

Number words (*one, two, three…*) and digits (*1, 2, 3…*) are both accepted.

### Camera calibration

After physically mounting the Kinect, update the camera-to-robot-base offsets in `kinect_picking.launch.py`:

```python
'cam_tx': 0.5,   # metres: camera X offset from robot base
'cam_ty': 0.1,   # metres: camera Y offset from robot base
'cam_tz': -0.2,  # metres: camera Z offset from robot base
```

Measure these values with:

```bash
ros2 run tf2_ros tf2_echo world kinect_rgb
```

---

## ROS 2 Topics & Parameters

### Topics

| Topic | Type | Direction | Description |
|---|---|---|---|
| `/kinect/rgb` | `sensor_msgs/Image` | Pub | RGB frames (bgr8, 640×480) |
| `/kinect/depth` | `sensor_msgs/Image` | Pub | Depth frames (32FC1, metres) |
| `/voice_command` | `std_msgs/String` | Sub/Pub | Voice target, e.g. `"circle_1"` |
| `/detection/pixel` | `geometry_msgs/PointStamped` | Pub | Detected shape pixel (u, v) |
| `/detection/pixel_with_depth` | `geometry_msgs/PointStamped` | Pub | Pixel + depth (u, v, Z) |
| `/detection/coords` | `geometry_msgs/PointStamped` | Pub | Camera-frame 3-D coords |
| `/detection/target_coords` | `geometry_msgs/PointStamped` | Pub | Voice-selected target 3-D coords |
| `/scaled_joint_trajectory_controller/joint_trajectory` | `trajectory_msgs/JointTrajectory` | Pub | Arm joint commands |
| `/move_group` | Action | Client | MoveIt 2 Cartesian moves |

### robot_commander_node Parameters

| Parameter | Default | Description |
|---|---|---|
| `cam_tx` | `0.0` | Camera X offset from robot base [m] |
| `cam_ty` | `0.0` | Camera Y offset from robot base [m] |
| `cam_tz` | `0.0` | Camera Z offset from robot base [m] |
| `pre_grasp_offset` | `0.10` | Height above object for pre-grasp approach [m] |
| `grasp_z_offset` | `0.00` | Fine-tune grasp depth [m] |
| `gripper_open` | `0.085` | Robotiq 2F fully open position [m] |
| `gripper_close` | `0.000` | Robotiq 2F fully closed position [m] |
| `move_duration` | `5.0` | Seconds per joint-space move [s] |
| `planning_group` | `ur_manipulator` | MoveIt 2 planning group name |
| `ee_link` | `tool0` | End-effector link name |
| `reference_frame` | `world` | MoveIt 2 reference frame |
| `tool_roll` | `1.5708` | EE roll for picks (π/2 = horizontal) [rad] |
| `tool_pitch` | `0.0` | EE pitch for picks [rad] |
| `tool_yaw` | `0.0` | EE yaw for picks [rad] |
| `ori_tolerance` | `0.10` | IK orientation tolerance (use `0.5` in sim) [rad] |

---

## Node Reference

### `kinect_node`
Real Kinect 360 driver using **libfreenect**. Publishes 15 Hz RGB (bgr8) and metric depth (32FC1) frames. Includes automatic retry on read failures.

### `mock_kinect_node`
Simulation drop-in for `kinect_node`. Publishes synthetic BGR frames containing three animated coloured shapes (pink circle, green square, pink triangle) at a fixed depth of 0.65 m. Zero hardware required.

### `detection_node`
Subscribes to `/kinect/rgb`. Runs HSV colour segmentation (pink: H 140–180 + H 0–10, green: H 35–85) followed by contour analysis and `approxPolyDP` shape classification. Assigns colour+shape+instance labels (e.g. `Pink_Circle_1`). When a voice target is active, highlights the matching object in cyan and publishes its stored 3-D coords to `/detection/target_coords`. Displays a live annotated OpenCV window.

### `depth_extractor_node`
Subscribes to `/kinect/depth` and `/detection/pixel`. For each detected pixel, extracts a 35-pixel median patch from the aligned depth image and emits `(u, v, Z_median)` on `/detection/pixel_with_depth`.

### `coord_transform_node`
Converts `(u, v, Z)` to camera-frame 3-D coordinates using the pinhole model:

```
X = (u - cx) * Z / fx
Y = (v - cy) * Z / fy
```

Loads calibration intrinsics from `~/.ros/camera_info/kinect_calibration.npz` if available; falls back to `fx = fy = 525, cx = 319.5, cy = 239.5`.

### `robot_commander_node`
The core orchestrator. Converts camera-frame coords to robot-base frame, then executes a full 9-step pick-and-place sequence using MoveIt 2 Cartesian planning and direct joint trajectory publishing. Includes thread-safe execution lock, duplicate-pick guard, and safety-home recovery on failure.

### `voice_command_node`
Listens to the default microphone via `speech_recognition` at 3-second intervals. Translates spoken phrases such as *"circle two"* or *"triangle 1"* into `circle_2` / `triangle_1` string messages on `/voice_command`. Supports both digit and word-form numbers (one–ten).

---

## Configuration

### Adjusting detection colours

Edit the HSV ranges in `detection_node.py`:

```python
# Pink  (H 0–10 and H 140–180, fully saturated)
pink1 = cv2.inRange(hsv, np.array([140, 30, 30]), np.array([180, 255, 255]))
pink2 = cv2.inRange(hsv, np.array([0,   30, 30]), np.array([10,  255, 255]))

# Green (H 35–85)
green = cv2.inRange(hsv, np.array([35, 30, 30]), np.array([85, 255, 255]))
```

### Adding new shapes / colours

The shape classifier in `detection_node.py` uses `approxPolyDP` vertex counting:

| Vertices | Shape |
|---|---|
| 3 | Triangle |
| 4 (aspect ratio ≈ 1) | Square |
| 4 (aspect ratio ≠ 1) | Rectangle |
| ≥ 5 (circularity > 0.70) | Circle |

### Pick-and-place timing

All timing parameters are exposed as ROS 2 node parameters (see table above) and can be overridden at launch without recompiling.

---

## Team

| Name | Role |
|---|---|
| **Jomana** | Team Leader — system integration & project coordination |
| **Youssef** | Director — robot motion planning & MoveIt 2 integration |
| **Tarek Okasha** | Voice module — speech recognition, command routing & vision pipeline |

*GIU Cairo — Intelligent Robotics Project, Spring 2026*

---

## License

This project is licensed under the **MIT License** — see the [LICENSE](LICENSE) file for details.

---

<div align="center">

Made with care at **GIU**

</div>
