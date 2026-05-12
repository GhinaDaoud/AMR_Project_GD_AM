"""
mission_web_server.py — Flask web server + ROS2 bridge for the mission GUI
Package : mission_planner

Run:  ros2 run mission_planner mission_web
Open: http://localhost:5000
"""

import json
import os
import queue
import threading

import rclpy
from ament_index_python.packages import get_package_share_directory
from flask import Flask, Response, jsonify, request, send_from_directory
from rclpy.node import Node
from std_msgs.msg import String


def _maps_dir() -> str:
    share = get_package_share_directory('my_local')
    ws_root = os.path.abspath(os.path.join(share, '..', '..', '..', '..'))
    return os.path.join(ws_root, 'src', 'my_local', 'maps')


def _static_dir() -> str:
    return os.path.join(get_package_share_directory('mission_planner'), 'web_static')


class MissionWebNode(Node):
    def __init__(self):
        super().__init__('mission_web_server')
        self._pub = self.create_publisher(String, '/mission_goal', 10)
        self._status_queues: list[queue.Queue] = []
        self._lock = threading.Lock()
        self.create_subscription(String, '/mission_status', self._on_status, 10)

    def _on_status(self, msg: String):
        with self._lock:
            for q in self._status_queues:
                q.put(msg.data)

    def send_goal(self, sequence: str):
        self._pub.publish(String(data=sequence))

    def subscribe_status(self) -> queue.Queue:
        q: queue.Queue = queue.Queue(maxsize=100)
        with self._lock:
            self._status_queues.append(q)
        return q

    def unsubscribe_status(self, q: queue.Queue):
        with self._lock:
            try:
                self._status_queues.remove(q)
            except ValueError:
                pass


# ── Flask app ─────────────────────────────────────────────────────────

app = Flask(__name__, static_folder=None)
_node: MissionWebNode | None = None
_landmarks: list[tuple[str, dict]] = []


@app.route('/')
def index():
    return send_from_directory(_static_dir(), 'index.html')


@app.route('/<path:filename>')
def static_files(filename):
    return send_from_directory(_static_dir(), filename)


@app.route('/api/landmarks')
def get_landmarks():
    result = []
    for i, (name, data) in enumerate(_landmarks, start=1):
        result.append({
            'id': i,
            'name': name,
            'x': data.get('x', 0.0),
            'y': data.get('y', 0.0),
        })
    return jsonify(result)


@app.route('/api/mission/start', methods=['POST'])
def start_mission():
    body = request.get_json(silent=True) or {}
    sequence = body.get('sequence', '')
    if not sequence:
        return jsonify({'error': 'sequence required'}), 400
    _node.send_goal(sequence)
    return jsonify({'ok': True, 'sequence': sequence})


@app.route('/api/mission/status')
def mission_status_stream():
    q = _node.subscribe_status()

    def generate():
        try:
            yield 'data: {"type":"connected"}\n\n'
            while True:
                try:
                    status = q.get(timeout=20.0)
                    payload = json.dumps({'type': 'status', 'value': status})
                    yield f'data: {payload}\n\n'
                except queue.Empty:
                    yield 'data: {"type":"ping"}\n\n'
        finally:
            _node.unsubscribe_status(q)

    return Response(generate(), mimetype='text/event-stream',
                    headers={'Cache-Control': 'no-cache',
                             'X-Accel-Buffering': 'no'})


# ── entry point ───────────────────────────────────────────────────────

def main(args=None):
    global _node, _landmarks

    rclpy.init(args=args)
    _node = MissionWebNode()

    db_path = os.path.join(_maps_dir(), 'landmark_db.json')
    with open(db_path, 'r') as f:
        _landmarks = list(json.load(f).items())

    threading.Thread(target=rclpy.spin, args=(_node,), daemon=True).start()

    _node.get_logger().info('Mission web server starting on http://localhost:5000')
    app.run(host='0.0.0.0', port=5000, threaded=True)

    _node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()