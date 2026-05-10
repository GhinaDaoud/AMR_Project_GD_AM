# AMR-EECE698 Project — Progress Summary

> **Course:** EECE698 — Autonomous Mobile Robots  
> **Student:** Ghina Daoud  
> **Platform:** ROS 2 Jazzy + Gazebo Harmonic, running in WSL2 on Windows 11

---

## Project Goal

Build a fully autonomous service robot that can:
1. **Part 1** — Explore an unknown maze, build a 2D SLAM map, and detect/record QR-code landmarks
2. **Part 2** — Navigate a dynamic environment (moving obstacles) and execute delivery missions between named locations
3. **Part 3** — (Future) Higher-level mission logic, multi-goal planning, human interaction

---

## Part 1 — Status: ✅ Complete

### Requirements vs. Implementation

| Requirement | Status | Implementation |
|-------------|--------|---------------|
| Autonomous exploration | ✅ | `service_robot/explorer.py` — reactive EXPLORE/TURN/BACKUP state machine using `/scan` |
| 2D SLAM mapping | ✅ | `slam_toolbox` `sync_slam_toolbox_node`, mapping mode |
| Save occupancy grid | ✅ | `service_robot/map_auto_saver.py` — subscribes to `/map`, writes PGM+YAML directly every 60 s |
| Save SLAM posegraph | ✅ | `map_auto_saver.py` calls `/slam_toolbox/serialize_map` (SerializePoseGraph) every 60 s → `.posegraph` + `.data` |
| QR code detection | ✅ | `landmark_detector/qr_detector.py` using `cv2.QRCodeDetector` |
| Landmark database | ✅ | `src/my_local/maps/landmark_db.json` — structured JSON with map-frame coordinates |
| Teleop option | ✅ | `mode:=teleop` launch argument disables explorer; user runs `teleop_twist_keyboard` |

### Confirmed Working (from live run)
```json
"landmarks": {
  "Zone-C: pillar B (0, -4)": { "x": 3.606, "y": 2.002, "frame_id": "map", "detection_count": 4 },
  "Zone-D: diagonal block (6, 6)": { "x": 13.171, "y": 14.553, "frame_id": "map", "detection_count": 1 }
}
```

---

## Part 2 — Dynamic Environment (Step 3, Ghina's task)

### Status: 🟡 Implemented — Pending Full Test

Step 3 is the addition of two independently wandering traffic robots to the maze. Steps 1, 2, 4, 5, 6 are teammate responsibilities.

| Sub-task | Status | Notes |
|----------|--------|-------|
| Traffic robot models (SDF) | ✅ | Two separate model folders in `worlds/` |
| Blue box robot (traffic_robot_1) | ✅ | 0.40×0.40×0.16 m chassis, dual casters, beacon tower |
| Green box robot (traffic_robot_2) | ✅ | 0.45×0.35×0.16 m chassis, dual casters, beacon tower |
| Included in maze world | ✅ | `maze.sdf` includes both via `model://` URI |
| Safe spawn positions | ✅ | robot_1 at (-7, 3), robot_2 at (-3, 5) — clear of all walls and QR landmarks |
| Reactive wanderer node | ✅ | `service_robot/traffic_wanderer.py` — same EXPLORE/TURN/BACKUP logic as explorer, parameterized by `robot_name` |
| ROS↔Gazebo bridges | ✅ | `traffic_bridge` node in launch — bridges `/traffic_robot_N/cmd_vel` and `/traffic_robot_N/scan` |
| Auto-started in launch | ✅ | Two `TimerAction` at t=15 s, one instance per robot |
| Speed tuned | ✅ | robot_1: 0.375 m/s linear, 0.825 rad/s turn; robot_2: 0.30 m/s, 0.975 rad/s |
| Mutual LIDAR detection | ✅ | Beacon towers (0.10×0.10×0.24 m, z=0.23→0.47) give each robot a target at the LIDAR scan height (z=0.24) |
| Mutual collision avoidance | ✅ | Beacon collision geometry + existing EXPLORE/TURN/BACKUP state machine |
| Main robot detects traffic bots | ✅ | Main LIDAR physically sees traffic robot bodies + beacons; explorer reacts and steers around them |
| SLAM map quality with moving bots | ⚠️ | Ghost traces expected in map where traffic robots roam — inherent SLAM limitation; resolved by Nav2 local costmap (teammate task) |

### How traffic robots are wired

```
maze.sdf
  └── <include> traffic_robot_a  →  model://traffic_robot_a
  └── <include> traffic_robot_b  →  model://traffic_robot_b

slam.launch.py
  ├── traffic_bridge         → bridges /traffic_robot_N/cmd_vel + /traffic_robot_N/scan
  ├── traffic_wanderer_1     → robot_name=traffic_robot_1, v=0.375, w=0.825
  └── traffic_wanderer_2     → robot_name=traffic_robot_2, v=0.30,  w=0.975

traffic_wanderer.py
  ├── subscribes  /{robot_name}/scan
  ├── publishes   /{robot_name}/cmd_vel
  └── state machine: EXPLORE → TURN → BACKUP → EXPLORE
```

### Why beacon towers?
The traffic robot LIDAR scans horizontally at z=0.24 m. The chassis top sits at z=0.23 m — a 1 cm gap means horizontal rays clear right over each other's chassis. The beacon (a 0.10×0.10×0.24 m box, z=0.23→0.47) ensures the LIDAR plane intersects the other robot's geometry. Gazebo ignores self-model collisions, so there are no self-hit issues.

---

## Part 2 — Remaining Steps (Teammate / Not Started)

| Step | Owner | Status |
|------|-------|--------|
| Step 1: Nav2 navigation stack on saved map | Teammate | ⬜ Not started |
| Step 2: Mission planner node (NavigateToPose) | Teammate | ⬜ Not started |
| Step 4: Python GUI (tkinter) for mission selection | TBD | ⬜ Not started |
| Step 5: Town world — add QR codes to mixed_town.world | TBD | ⬜ Not started |
| Step 6: SLAM mapping run on town world | TBD | ⬜ Not started |

---

## Data Output Location

All Part 1 outputs land in one folder, accessible from Windows File Explorer:

```
Project/src/my_local/maps/
  maze_map.pgm          ← occupancy grid image (open in Paint/Photos)
  maze_map.yaml         ← map metadata (resolution, origin, pgm path)
  maze_map.posegraph    ← SLAM internal graph (resume sessions)
  maze_map.data         ← SLAM keyframe data
  landmark_db.json      ← landmark names + map-frame positions
```

### Data Formats Chosen (stable for Part 2)

| Data | Format | Why |
|------|--------|-----|
| Occupancy map | `.pgm + .yaml` | ROS2 standard — directly loaded by `nav2_map_server` |
| SLAM graph | `.posegraph + .data` | slam_toolbox native — resume or re-localise |
| Landmarks | `.json` | Human-readable, Python-native, loaded directly by `mission_planner` |

### landmark_db.json Schema
```json
{
  "_meta": { "version": "1.0", "frame_id": "map", "description": "..." },
  "landmarks": {
    "<QR text>": {
      "x": 3.606,
      "y": 2.002,
      "frame_id": "map",
      "first_seen": "ISO-8601 UTC",
      "last_updated": "ISO-8601 UTC",
      "detection_count": 4
    }
  }
}
```

---

## How to Run

### Build
```bash
cd /mnt/c/Users/user/OneDrive\ -\ American\ University\ of\ Beirut/Courses/AMR-EECE698/Project
colcon build --packages-select service_robot landmark_detector robot_description my_local
source install/setup.bash
```

### Launch — Autonomous mode (explorer + traffic robots)
```bash
ros2 launch robot_description slam.launch.py
# or explicitly:
ros2 launch robot_description slam.launch.py mode:=auto
```

### Launch — Teleop mode (you drive)
```bash
# Terminal 1
ros2 launch robot_description slam.launch.py mode:=teleop

# Terminal 2 (after simulation starts)
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### Startup Sequence
| Time | Event |
|------|-------|
| 0 s | Gazebo, all bridges, robot_state_publisher start |
| 5 s | Main robot spawns at (-7, -7); traffic robots already in world at (-7,3) and (-3,5) |
| 8 s | SLAM toolbox starts mapping |
| 10 s | RViz opens (Map + RobotModel + LaserScan) |
| 12 s | Explorer starts driving / QR detector starts watching camera |
| 15 s | traffic_wanderer_1 and traffic_wanderer_2 start — both robots begin wandering |
| 70 s | `map_auto_saver` starts — first save fires immediately |
| every 60 s | maze_map.pgm + .yaml + .posegraph + .data all written to maps/ |

### Debug traffic robots
```bash
ros2 topic echo /traffic_robot_1/scan --no-arr   # confirm LIDAR data
ros2 topic echo /traffic_robot_1/cmd_vel          # confirm wanderer is sending commands
ros2 topic echo /traffic_robot_2/cmd_vel
```

---

## Package Structure

```
src/
  robot_description/        ← CMake package — URDF, Gazebo worlds, launch files
    urdf/
      service_robot.urdf.xacro    ← robot geometry (chassis + drive wheels + caster)
      gazebo_plugins.xacro        ← DiffDrive, sensors, caster friction
    worlds/
      maze.sdf                    ← 20×20 m maze with 4 QR landmarks + 2 traffic robots
      qr_textures/                ← PNG QR images for each landmark
      traffic_robot_a/            ← Blue box traffic robot model
        model.sdf
        model.config
      traffic_robot_b/            ← Green box traffic robot model
        model.sdf
        model.config
    config/
      slam_config.yaml            ← slam_toolbox params (use_map_saver: false)
      slam.rviz                   ← pre-configured RViz
    launch/
      slam.launch.py              ← main launch file (mode:=auto|teleop)

  service_robot/            ← Python package — robot behaviour
    service_robot/
      explorer.py                 ← reactive autonomous explorer (EXPLORE/TURN/BACKUP)
      map_auto_saver.py           ← periodic map saver (PGM+YAML + posegraph via slam_toolbox)
      traffic_wanderer.py         ← parameterized wanderer for traffic robots

  landmark_detector/        ← Python package — vision
    landmark_detector/
      qr_detector.py              ← QR detection, bounding-box overlay, DB save

  mission_planner/          ← Python package — Part 2 Step 2 (not yet implemented)

  my_local/                 ← stub package used for maps/ path resolution
    maps/                   ← ALL output data lands here
```

---

## Robot Designs

### Main Service Robot
| Property | Value |
|----------|-------|
| Chassis | 0.45 × 0.30 × 0.15 m, blue box |
| Drive wheels | radius 0.07 m, width 0.05 m — rear (x = −0.12 m) |
| Front caster | sphere radius 0.035 m |
| Wheel separation | 0.35 m |
| LIDAR | 360° GPU lidar, 10 Hz, 0.15–12 m range |
| Camera | 640×480, 60° HFOV, 10 Hz |
| IMU | 50 Hz |

### Traffic Robot A (Blue)
| Property | Value |
|----------|-------|
| Chassis | 0.40 × 0.40 × 0.16 m, bright blue |
| Beacon tower | 0.10 × 0.10 × 0.24 m, darker blue — spans z=0.23→0.47 for LIDAR detection |
| Drive wheels | radius 0.07 m — at y = ±0.22 m |
| Casters | front (x=+0.15) + rear (x=−0.15) — flat, stable stance |
| Wheel separation | 0.44 m |
| LIDAR | 180 samples, 10 Hz, 0.15–8 m, scans at z=0.24 m |
| Speed | 0.375 m/s linear, 0.825 rad/s turn |

### Traffic Robot B (Green)
| Property | Value |
|----------|-------|
| Chassis | 0.45 × 0.35 × 0.16 m, bright green |
| Beacon tower | 0.10 × 0.10 × 0.24 m, darker green — spans z=0.23→0.47 for LIDAR detection |
| Drive wheels | radius 0.07 m — at y = ±0.195 m |
| Casters | front (x=+0.17) + rear (x=−0.17) — flat, stable stance |
| Wheel separation | 0.39 m |
| LIDAR | 180 samples, 10 Hz, 0.15–8 m, scans at z=0.24 m |
| Speed | 0.30 m/s linear, 0.975 rad/s turn |

---

## Key Bugs Fixed During Development

| Bug | Root Cause | Fix |
|-----|-----------|-----|
| Robot invisible in RViz | RobotModel topic missing `Durability: Transient Local` QoS | Added to `slam.rviz` config |
| Robot clips through floor | `base_footprint_joint z = wheel_radius` (too low) | Changed to `wheel_radius + base_height/2` |
| Caster floats above ground | Caster z offset too shallow | Fixed to `-(base_height/2 + caster_radius)` |
| Odometry drift | `wheel_separation = 0.38` but actual geometry = 0.35 | Corrected in DiffDrive plugin |
| Wheel inertia error | `ixx + iyy < izz` violated triangle inequality | Reverted to physically valid values |
| Map not saving | `map_saver_server` started but never triggered | Replaced with `map_auto_saver.py` that saves directly from `/map` topic every 60 s |
| Posegraph save result=255 | `use_map_saver: true` made slam_toolbox call `map_saver_cli` internally, which fails on paths with spaces (OneDrive path) | Set `use_map_saver: false`; use `/slam_toolbox/serialize_map` (SerializePoseGraph) directly |
| SLAM startup "Failed to open file" | `map_file_name` param told SLAM to load a non-existent file | Removed `map_file_name` from launch |
| pyzbar crash | `pyzbar` not installed in WSL | Replaced with `cv2.QRCodeDetector` |
| QR window closing on detection | Unhandled exception skipped `cv2.imshow()` | Wrapped in `try/except`; imshow always runs |
| QR textures not showing | `maze.sdf` hardcoded `/home/test/` path | Launch file patches SDF with correct path at runtime using `tempfile` |
| Landmark path wrong | `~/AMR_project/` symlink assumed | Use `get_package_share_directory` walk-up for real workspace path |
| RViz TF error | Missing `/joint_states` bridge for wheel joints | Added `joint_state_bridge` node |
| SLAM drops scans | `transform_timeout = 0.2 s` too short | Increased to `0.5 s` |
| Explorer not found | `setup.py` had no entry points | Added all executables to `console_scripts` |
| QR codes too high for camera | QR faces at 0.75–1 m, camera at 0.145 m | Repositioned to world z ≈ 0.35 m |
| QR codes too large | Full face 1.4–2.9 m wide | Halved to 0.7 m × 0.7 m |
| Gazebo entirely black window | `<render_engine>ogre2</render_engine>` fails in WSL2 | Changed to `<render_engine>ogre</render_engine>` in `maze.sdf` |
| traffic_robot_1 stuck / not moving | Spawned at (3, 0) which is on `h_div_left` wall (y=0 corridor wall) | Moved spawn to (-7, 3) — confirmed clear of all walls |
| traffic_robot_2 invisible | Spawned at (0, 3) which is inside `pillar_a` QR landmark (1.5×1.5×2 m box) | Moved spawn to (-3, 5) — confirmed clear |
| Traffic robots clip through each other | Chassis top at z=0.23, LIDAR at z=0.24 — rays clear over each other | Added beacon tower to each robot; collision spans z=0.23→0.47 so LIDAR sees it |
| traffic_robot_b chassis rocks/tilts | Only one front caster — no rear support under deceleration | Added rear caster to both traffic robots |
| traffic_robot_a cylinder design | Cylinder body visually unclear and physically awkward | Replaced with flat box chassis matching overall robot style |

---

## Environment Notes

| Item | Value |
|------|-------|
| OS | Windows 11, WSL2 (Ubuntu) |
| ROS2 | Jazzy |
| Gazebo | Harmonic (gz-sim) |
| Render engine | `ogre` (ogre2 causes black screen in WSL2) |
| Workspace root (WSL) | `/mnt/c/Users/user/OneDrive - American University of Beirut/Courses/AMR-EECE698/Project` |
| Maps output | `src/my_local/maps/` |
| Pylance warning on `ament_index_python` | Harmless — Windows Python has no ROS2; works correctly in WSL |
