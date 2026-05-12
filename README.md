# Autonomous Service Robot — Simulated Town Navigation

A ROS2-based autonomous mobile robot that maps a simulated town using SLAM, detects QR-coded landmarks with a camera, and executes multi-stop delivery missions through a web GUI.

---

## Demo

<!-- Add screenshots or a demo video link here -->
> 📸 Place screenshots in `docs/images/` and reference them below.
> 🎥 For videos, upload to YouTube and paste the link.
https://youtu.be/ttTxQ7kZ4OM

| Mapping Phase | Navigation Phase | Web GUI |
|:---:|:---:|:---:|
| ![SLAM Map](docs/images/slam_map.png) | ![Nav2 Path](docs/images/nav2_path.png) | ![Web GUI](docs/images/web_gui.png) |

---

## System Overview

```
Gazebo Simulation
      │
      ├─ LiDAR → SLAM Toolbox → /map
      ├─ Camera → QR Detector → landmark_db.json
      └─ /cmd_vel ← Nav2 ← Mission Executor ← Flask Web GUI
```

---

## Features

- **Autonomous SLAM mapping** with a reactive explorer (auto + keyboard override)
- **QR landmark detection** using dual cv2 + pyzbar decoder; saves positions in map frame via TF
- **Nav2 navigation** with static-map-only costmap (no lidar noise artifacts)
- **Dynamic obstacle avoidance** using Nav2 live costmap layer — robot detects and re-routes around unexpected obstacles in real time
- **Mission planner** that sends the robot to user-selected landmarks in sequence and docks it back
- **Flask web GUI** with role-based landmark icons and real-time status via Server-Sent Events

---

## Town Layout

The simulated environment covers 40 × 40 m and contains 9 landmarks:

| Landmark | Type |
|---|---|
| 🛒 Supermarket | Commercial |
| 🍽️ Restaurant | Commercial |
| 💊 Pharmacy | Service |
| 🚒 Fire Station | Service |
| 🏠 House 1–5 | Residential |

Each landmark has a **physical QR code sign** placed at its entrance. The robot reads these during mapping to build the navigation database.

---

## Architecture

### Packages

| Package | Purpose |
|---|---|
| `robot_description` | URDF, Gazebo world, Nav2/SLAM configs, launch files |
| `landmark_detector` | QR code detection node |
| `mission_planner` | Nav2 action client + Flask web server |
| `service_robot` | Explorer, teleop mux, map auto-saver |
| `my_local` | Shared map files and landmark database |

### Robot Hardware (Simulated)

- Differential drive with rear drive wheels + front caster
- 360° GPU LiDAR (12 m range, 720 rays)
- RGB camera (640×480)
- IMU

---

## Prerequisites

- ROS2 Jazzy
- Gazebo Harmonic (Gz Sim 8)
- Python packages: `flask`, `qrcode`, `pyzbar`, `opencv-python`, `pillow`

```bash
pip install flask qrcode pyzbar opencv-python pillow
```

---

## Phase 1 — SLAM Mapping

```bash
source install/setup.bash

# Hybrid mode (auto explorer + keyboard override)
ros2 launch robot_description town_slam.launch.py

# Continue from a previous session
ros2 launch robot_description town_slam.launch.py continue_mapping:=true

# Teleop only
ros2 launch robot_description town_slam.launch.py mode:=teleop
```

**Keyboard override (hybrid mode):** Press any movement key in the xterm window to take control. Release for 0.5 s and the explorer resumes automatically.

### Force-save the map

```bash
# Save posegraph (.posegraph + .data)
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
  "{filename: '/home/test/phases/src/my_local/maps/town_map'}"

# Save occupancy grid (.pgm + .yaml)
ros2 run nav2_map_server map_saver_cli \
  -f /home/test/phases/src/my_local/maps/town_map \
  --ros-args -p use_sim_time:=true
```

---

## Phase 2 — Autonomous Navigation

```bash
source install/setup.bash
ros2 launch robot_description town_nav2.launch.py
```

Then open the mission GUI at **http://localhost:5000**

1. Select landmarks in delivery order
2. Click **Start Mission**
3. Robot navigates to each stop and returns to the docking station

---

## Dynamic Obstacle Avoidance

The robot handles unexpected obstacles at runtime using Nav2's live obstacle layer on top of the static map. When a new obstacle is detected by the LiDAR mid-mission:

1. The **local costmap** marks the obstacle in real time
2. The **global planner** is triggered to recompute the path around it
3. The **regulated pure pursuit controller** follows the updated path
4. Once the obstacle is cleared, the costmap decays and the original path is restored

This was developed as a collaborative extension to the base navigation stack.

> 📂 Code for this feature will be added to the repository shortly.

---

## Key Design Decisions

| Problem | Solution |
|---|---|
| Robot tipping backwards | Drive wheels moved 5 cm rearward — CoM inside support triangle |
| All QR signs showing same texture | Unique PNG filename per sign defeats Gazebo texture cache |
| Gazebo freeze on fire station QR | `pyzbar` only called when `cv2` finds nothing |
| Landmark coordinates wrong (odom drift) | QR detector uses TF `map → base_footprint` instead of `/odom` |
| Phantom obstacles blocking paths | Static-map-only costmap — no live LiDAR obstacle layer |
| Map lost on Gazebo close | Manual force-save commands; Gazebo server/GUI split option |
| Robot stuck in dead-ends | Hybrid teleop mux — keyboard overrides explorer instantly |

---

## Project Structure

```
phases/
├── src/
│   ├── robot_description/     # URDF, configs, launch files
│   ├── landmark_detector/     # QR detection node
│   ├── mission_planner/       # Nav2 client + Flask GUI
│   ├── service_robot/         # Explorer, mux, map saver
│   └── my_local/              # Shared maps and landmark DB
└── town/
    ├── mixed_town.world        # Gazebo world
    └── models/                # Buildings, signs, QR textures
```

---

## Adding Media

Place screenshots and images in `docs/images/` and update the table at the top of this file.
For videos, upload to YouTube and add the link or embed badge here.
