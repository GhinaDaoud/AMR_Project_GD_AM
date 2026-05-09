# AMR-EECE698 Project — Progress Summary

> **Course:** EECE698 — Autonomous Mobile Robots  
> **Student:** Ghina Daoud  
> **Platform:** ROS 2 Jazzy + Ignition Gazebo (Harmonic), running in WSL2 on Windows 11

---

## Project Goal

Build a fully autonomous service robot that can:
1. **Part 1** — Explore an unknown maze, build a 2D SLAM map, and detect/record QR-code landmarks
2. **Part 2** — Use the saved map and landmark database to execute delivery missions between named locations
3. **Part 3** — (Future) Higher-level mission logic, multi-goal planning, human interaction

---

## Part 1 — Status: ✅ Complete

### Requirements vs. Implementation

| Requirement | Status | Implementation |
|-------------|--------|---------------|
| Autonomous exploration | ✅ | `service_robot/explorer.py` — reactive EXPLORE/TURN/BACKUP state machine using `/scan` |
| 2D SLAM mapping | ✅ | `slam_toolbox` `sync_slam_toolbox_node`, mapping mode |
| Save occupancy grid | ✅ | `service_robot/map_auto_saver.py` — subscribes to `/map`, writes PGM+YAML directly every 60s |
| Save SLAM posegraph | ✅ | `map_auto_saver.py` calls `/slam_toolbox/save_map` service every 60s → `.posegraph` + `.data` |
| QR code detection | ✅ | `landmark_detector/qr_detector.py` using `cv2.QRCodeDetector` |
| Landmark database | ✅ | `src/my_local/maps/landmark_db.json` — structured JSON with map-frame coordinates |
| Reuse in Part 2 | ✅ | Map loaded via `map_server`, landmarks loaded by `mission_planner` from JSON |
| Teleop option | ✅ | `mode:=teleop` launch argument disables explorer; user runs `teleop_twist_keyboard` |

### Confirmed Working (from live run)
```json
"landmarks": {
  "Zone-C: pillar B (0, -4)": { "x": 3.606, "y": 2.002, "frame_id": "map", "detection_count": 4 },
  "Zone-D: diagonal block (6, 6)": { "x": 13.171, "y": 14.553, "frame_id": "map", "detection_count": 1 }
}
```

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
      "x": 3.606,           ← map-frame X (metres)
      "y": 2.002,           ← map-frame Y (metres)
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

### Launch — Autonomous mode (explorer drives)
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
| 0s | Gazebo, bridges, robot_state_publisher start |
| 5s | Robot spawns at (-7, -7) in the maze |
| 8s | SLAM toolbox starts mapping |
| 10s | RViz opens (Map + RobotModel + LaserScan pre-configured) |
| 12s | Explorer starts driving / QR detector starts watching camera |
| 70s | `map_auto_saver` starts — first save fires immediately |
| every 60s | maze_map.pgm + .yaml + .posegraph + .data all written to maps/ |

### Manual Map Save (anytime while running)
```bash
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \
  "{name: {data: '/mnt/c/Users/user/OneDrive - American University of Beirut/Courses/AMR-EECE698/Project/src/my_local/maps/maze_map'}}"
```

---

## Package Structure

```
src/
  robot_description/        ← CMake package — robot URDF, Gazebo worlds, launch files
    urdf/
      service_robot.urdf.xacro    ← robot geometry (chassis + rear drive wheels + front caster)
      gazebo_plugins.xacro        ← DiffDrive, sensors, caster friction
    worlds/
      maze.sdf                    ← 20×20m maze with 4 QR landmark boxes
      qr_textures/                ← PNG QR code images for each landmark
    config/
      slam_config.yaml            ← slam_toolbox parameters
      slam.rviz                   ← pre-configured RViz (Map, RobotModel, LaserScan)
    launch/
      slam.launch.py              ← main launch file (mode:=auto|teleop)

  service_robot/            ← Python package — robot behaviour
    service_robot/
      explorer.py                 ← reactive autonomous explorer (EXPLORE/TURN/BACKUP)
      map_auto_saver.py           ← periodic map saver (PGM+YAML from /map, posegraph via slam_toolbox service)

  landmark_detector/        ← Python package — vision
    landmark_detector/
      qr_detector.py              ← QR detection, bounding-box overlay, DB save

  mission_planner/          ← Python package — Part 2 (not yet implemented)

  my_local/                 ← stub package used for maps/ path resolution
    maps/                   ← ALL output data lands here
```

---

## Robot Design

Based on the Gazebo tutorial differential-drive car proportions.

| Property | Value | Notes |
|----------|-------|-------|
| Chassis | 0.45 × 0.30 × 0.15 m | Blue box |
| Drive wheels | radius 0.07 m, width 0.05 m | Dark gray, at **rear** (x = −0.12 m) |
| Front caster | sphere radius 0.035 m | Green, at front (x = +0.165 m) |
| Wheel separation | 0.35 m | Matches DiffDrive plugin exactly |
| LIDAR | 360° GPU lidar, 10 Hz, 0.15–12 m | On top of chassis |
| Camera | 640×480, 60° HFOV, 10 Hz | Front face, horizontal |
| IMU | 50 Hz | At chassis centre |

**Key URDF fixes applied:**
- `base_footprint_joint z = wheel_radius + base_height/2 = 0.145 m` — prevents chassis clipping floor
- Caster z = `-(base_height/2 + caster_radius) = -0.11 m` — sphere touches ground exactly
- Caster `mu1=mu2=0` in Gazebo — slides freely without fighting drive wheels
- Inertia tensors computed from geometry formulas — no more "invalid inertia" Gazebo errors

---

## Key Bugs Fixed During Development

| Bug | Root Cause | Fix |
|-----|-----------|-----|
| Robot invisible in RViz | RobotModel topic missing `Durability: Transient Local` QoS | Added to `slam.rviz` config |
| Robot clips through floor | `base_footprint_joint z = wheel_radius` (too low) | Changed to `wheel_radius + base_height/2` |
| Caster floats above ground | Caster z offset too shallow | Fixed to `-(base_height/2 + caster_radius)` |
| Odometry drift | `wheel_separation = 0.38` but actual geometry = 0.34/0.35 | Corrected in DiffDrive plugin |
| Wheel inertia error | `ixx + iyy < izz` violated triangle inequality | Reverted to physically valid values |
| Map not saving | `map_saver_server` was started but never called — nothing triggered the save | Replaced with `map_auto_saver.py` node that saves directly from `/map` topic every 60s |
| Posegraph save result=255 | `use_map_saver: true` made slam_toolbox try to call `map_saver_cli` internally, which failed | Set `use_map_saver: false` — posegraph-only save now succeeds |
| pyzbar crash | `pyzbar` not installed in WSL environment | Replaced with `cv2.QRCodeDetector` |
| QR window closing on detection | Unhandled exception in callback skipped `cv2.imshow()` | Wrapped all processing in `try/except`; imshow always runs |
| QR textures not showing | `maze.sdf` hardcoded `/home/test/` path | Launch file patches SDF with correct path at runtime |
| Landmark path wrong | `~/AMR_project/` symlink assumed (doesn't exist) | Use `get_package_share_directory` walk-up for real path |
| Map save `result=255` | `map_saver_server` in unconfigured lifecycle state | Lifecycle manager fix (see above) |
| RViz global status TF error | Missing `/joint_states` bridge for wheel joints | Added `joint_state_bridge` node |
| SLAM drops scans | `transform_timeout = 0.2s` too short | Increased to `0.5s` |
| Explorer not in package | `setup.py` had no entry points | Added `explorer = service_robot.explorer:main` |
| QR codes too high for camera | QR faces centered at 0.75–1 m, camera at 0.145 m | Repositioned to world z ≈ 0.35 m |
| QR codes too large | Full face size (1.4–2.9 m) | Halved to 0.7 m × 0.7 m (or 1.45 × 0.7) |

---

## Part 2 — Next Steps (Not Started)

`src/mission_planner/` exists as an empty stub. The mission planner needs to:

1. **Load saved data**
   - Map: `ros2 run nav2_map_server map_server --ros-args -p map_yaml_filename:=maze_map.yaml`
   - Landmarks: read `landmark_db.json` → build a name→(x,y) lookup table

2. **Navigate to landmarks**
   - Use Nav2 action client (`NavigateToPose`) with goals from the landmark DB
   - Requires: `nav2_bringup` (AMCL localisation + planners) loaded on top of the saved map

3. **Mission logic**
   - Accept a goal landmark name (e.g. `"Zone-C: pillar B (0, -4)"`)
   - Look up its (x, y) from the DB
   - Send a `NavigateToPose` action goal
   - Report success/failure

4. **Entry point** to add to `mission_planner/setup.py`:
   ```python
   'mission_planner = mission_planner.mission_node:main'
   ```

---

## Environment Notes

| Item | Value |
|------|-------|
| OS | Windows 11, WSL2 (Ubuntu) |
| ROS2 | Jazzy |
| Gazebo | Harmonic (gz-sim) |
| Workspace root (WSL) | `/mnt/c/Users/user/OneDrive - American University of Beirut/Courses/AMR-EECE698/Project` |
| Maps output | `src/my_local/maps/` |
| Pylance warning on `ament_index_python` | **Harmless** — Windows Python has no ROS2; works correctly in WSL |
