"""
map_auto_saver.py — Periodic automatic map saver (Part 1)
Package : service_robot
Node    : map_auto_saver
Run via : ros2 launch robot_description slam.launch.py  (started at t=70s)
          ros2 run service_robot map_auto_saver          (standalone)

What it does
------------
Every 60 seconds, saves two things:

  1. Occupancy grid  →  maze_map.pgm + maze_map.yaml
     Subscribes to /map (nav_msgs/OccupancyGrid) and writes the PGM+YAML
     directly in Python. This avoids map_saver_cli which breaks on paths
     that contain spaces (the OneDrive folder name).

  2. SLAM posegraph  →  maze_map.posegraph + maze_map.data
     Calls /slam_toolbox/serialize_map (slam_toolbox/srv/SerializePoseGraph).
     These binary files let slam_toolbox resume or re-localise in a later
     session without re-exploring.

Why NOT use /slam_toolbox/save_map?
------------------------------------
  save_map saves the occupancy grid PGM by internally calling map_saver_cli.
  That CLI call fails with "Wrong argument: -" when the file path contains
  spaces. serialize_map writes directly in C++ and has no such issue.

Output files (all in src/my_local/maps/)
-----------------------------------------
  maze_map.pgm        — occupancy grid image (open in any image viewer)
  maze_map.yaml       — map metadata: resolution, origin, threshold values
  maze_map.posegraph  — SLAM graph used to resume mapping or re-localise
  maze_map.data       — SLAM keyframe dataset paired with .posegraph

Topics / services
-----------------
  Subscribes : /map                      (nav_msgs/OccupancyGrid)
  Service    : /slam_toolbox/serialize_map (slam_toolbox/srv/SerializePoseGraph)

Debug tips
----------
  Confirm saves    : ros2 topic echo /rosout | grep map_auto_saver
  Force a save now : wait for the 60 s timer, or restart the node
  Check file sizes : ls -lh src/my_local/maps/
"""

import os
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from slam_toolbox.srv import SerializePoseGraph


SAVE_INTERVAL_SEC = 60.0
FREE_THRESH = 0.25       # cells with p_occ < this → free (white 255)
OCC_THRESH  = 0.65       # cells with p_occ > this → occupied (black 0)
UNKNOWN_VAL = 205        # gray value for unknown cells in PGM


class MapAutoSaver(Node):

    def __init__(self, maps_dir: str):
        super().__init__('map_auto_saver')

        self._maps_dir = maps_dir
        self._map_base = os.path.join(maps_dir, 'maze_map')
        self._latest_map: OccupancyGrid | None = None

        self.create_subscription(OccupancyGrid, '/map', self._map_cb, 10)

        self._serialize_cli = self.create_client(
            SerializePoseGraph, '/slam_toolbox/serialize_map'
        )

        self.create_timer(SAVE_INTERVAL_SEC, self._save_all)

        self.get_logger().info(
            f'MapAutoSaver ready — saving every {SAVE_INTERVAL_SEC:.0f}s to {self._map_base}.*'
        )

    # ── callbacks ────────────────────────────────────────────────────────── #

    def _map_cb(self, msg: OccupancyGrid) -> None:
        self._latest_map = msg

    def _save_all(self) -> None:
        self._save_occupancy_grid()
        self._save_posegraph()

    # ── occupancy grid → PGM + YAML ──────────────────────────────────────── #

    def _save_occupancy_grid(self) -> None:
        if self._latest_map is None:
            self.get_logger().warn('No /map received yet — skipping occupancy grid save')
            return

        msg = self._latest_map
        width  = msg.info.width
        height = msg.info.height
        res    = msg.info.resolution
        ox     = msg.info.origin.position.x
        oy     = msg.info.origin.position.y

        # Build PGM pixel array (row-major, top-to-bottom in image = max-y first)
        pixels = bytearray(width * height)
        for row in range(height):
            for col in range(width):
                idx = (height - 1 - row) * width + col   # flip Y for image coords
                cell = msg.data[idx]
                if cell == -1:
                    pixels[row * width + col] = UNKNOWN_VAL
                elif cell / 100.0 > OCC_THRESH:
                    pixels[row * width + col] = 0          # occupied → black
                elif cell / 100.0 < FREE_THRESH:
                    pixels[row * width + col] = 255        # free → white
                else:
                    pixels[row * width + col] = UNKNOWN_VAL

        pgm_path  = self._map_base + '.pgm'
        yaml_path = self._map_base + '.yaml'

        # Write PGM (binary P5)
        with open(pgm_path, 'wb') as f:
            header = f'P5\n{width} {height}\n255\n'.encode()
            f.write(header)
            f.write(pixels)

        # Write YAML metadata
        yaml_content = (
            f'image: maze_map.pgm\n'
            f'resolution: {res}\n'
            f'origin: [{ox}, {oy}, 0.0]\n'
            f'negate: 0\n'
            f'occupied_thresh: {OCC_THRESH}\n'
            f'free_thresh: {FREE_THRESH}\n'
        )
        with open(yaml_path, 'w') as f:
            f.write(yaml_content)

        self.get_logger().info(
            f'Saved occupancy grid: {pgm_path} ({width}x{height} @ {res}m/px)'
        )

    # ── posegraph → .posegraph + .data ───────────────────────────────────── #

    def _save_posegraph(self) -> None:
        if not self._serialize_cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn(
                '/slam_toolbox/serialize_map not available — skipping posegraph save'
            )
            return

        req = SerializePoseGraph.Request()
        req.filename = self._map_base
        future = self._serialize_cli.call_async(req)
        future.add_done_callback(self._posegraph_done)

    def _posegraph_done(self, future) -> None:
        try:
            result = future.result()
            self.get_logger().info(f'Posegraph saved (result={result.result})')
        except Exception as exc:
            self.get_logger().error(f'Posegraph save failed: {exc}')


# ── entry point ──────────────────────────────────────────────────────────── #

def main(args=None):
    rclpy.init(args=args)

    from ament_index_python.packages import get_package_share_directory
    _share = get_package_share_directory('my_local')
    _ws_root = os.path.abspath(os.path.join(_share, '..', '..', '..', '..'))
    maps_dir = os.path.join(_ws_root, 'src', 'my_local', 'maps')
    os.makedirs(maps_dir, exist_ok=True)

    node = MapAutoSaver(maps_dir)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
