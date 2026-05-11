"""
map_auto_saver.py — Periodic automatic map saver (Part 1)
Package : service_robot
Node    : map_auto_saver

What it does
------------
Every 60 seconds saves:
  1. Occupancy grid  →  <map_name>.pgm + <map_name>.yaml
  2. SLAM posegraph  →  <map_name>.posegraph + <map_name>.data

ROS2 parameter
--------------
  map_name (string, default: "maze_map") — base filename for all saved files.
  Pass "town_map" in the town launch file.

Output: src/my_local/maps/<map_name>.*
"""

import os
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from slam_toolbox.srv import SerializePoseGraph


SAVE_INTERVAL_SEC = 60.0
FREE_THRESH = 0.25
OCC_THRESH  = 0.65
UNKNOWN_VAL = 205


class MapAutoSaver(Node):

    def __init__(self, maps_dir: str):
        super().__init__('map_auto_saver')

        self.declare_parameter('map_name', 'maze_map')
        map_name = self.get_parameter('map_name').get_parameter_value().string_value

        self._maps_dir = maps_dir
        self._map_base = os.path.join(maps_dir, map_name)
        self._latest_map: OccupancyGrid | None = None

        self.create_subscription(OccupancyGrid, '/map', self._map_cb, 10)
        self._serialize_cli = self.create_client(
            SerializePoseGraph, '/slam_toolbox/serialize_map'
        )
        self.create_timer(SAVE_INTERVAL_SEC, self._save_all)

        self.get_logger().info(
            f'MapAutoSaver ready — saving every {SAVE_INTERVAL_SEC:.0f}s to {self._map_base}.*'
        )

    def _map_cb(self, msg: OccupancyGrid) -> None:
        self._latest_map = msg

    def _save_all(self) -> None:
        self._save_occupancy_grid()
        self._save_posegraph()

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

        pixels = bytearray(width * height)
        for row in range(height):
            for col in range(width):
                idx = (height - 1 - row) * width + col
                cell = msg.data[idx]
                if cell == -1:
                    pixels[row * width + col] = UNKNOWN_VAL
                elif cell / 100.0 > OCC_THRESH:
                    pixels[row * width + col] = 0
                elif cell / 100.0 < FREE_THRESH:
                    pixels[row * width + col] = 255
                else:
                    pixels[row * width + col] = UNKNOWN_VAL

        pgm_path  = self._map_base + '.pgm'
        yaml_path = self._map_base + '.yaml'
        map_stem  = os.path.basename(self._map_base)

        with open(pgm_path, 'wb') as f:
            f.write(f'P5\n{width} {height}\n255\n'.encode())
            f.write(pixels)

        with open(yaml_path, 'w') as f:
            f.write(
                f'image: {map_stem}.pgm\n'
                f'resolution: {res}\n'
                f'origin: [{ox}, {oy}, 0.0]\n'
                f'negate: 0\n'
                f'occupied_thresh: {OCC_THRESH}\n'
                f'free_thresh: {FREE_THRESH}\n'
            )

        self.get_logger().info(
            f'Saved occupancy grid: {pgm_path} ({width}x{height} @ {res}m/px)'
        )

    def _save_posegraph(self) -> None:
        if not self._serialize_cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('/slam_toolbox/serialize_map not available — skipping')
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


def main(args=None):
    rclpy.init(args=args)

    from ament_index_python.packages import get_package_share_directory
    share = get_package_share_directory('my_local')
    ws_root = os.path.abspath(os.path.join(share, '..', '..', '..', '..'))
    maps_dir = os.path.join(ws_root, 'src', 'my_local', 'maps')
    os.makedirs(maps_dir, exist_ok=True)

    node = MapAutoSaver(maps_dir)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()