"""WebSocket server for streaming 3D point cloud data to Babylon.js viewer.

Binary protocol (outbound):
  0x01 = Full point cloud (Float32 xyz array)
  0x02 = Delta points (new points since last full send)
  0x03 = Trajectory (keyframe positions as Float32 xyz array)
  0x04 = Status JSON (UTF-8 encoded)
  0x14 = Nav status JSON (UTF-8 encoded) — game nav feedback

Text protocol (inbound):
  JSON: { "type": "nav_goal", "x": float, "y": float, "theta": float, "cache_id": str }

Delta streaming keeps bandwidth <50KB/s. Full refresh every 30s.
"""

import asyncio
import json
import math
import struct
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import LaserScan, PointCloud2
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid

try:
    import websockets
    from websockets.server import serve
except ImportError:
    websockets = None


# Protocol message types
MSG_FULL_CLOUD = 0x01
MSG_DELTA_POINTS = 0x02
MSG_TRAJECTORY = 0x03
MSG_STATUS = 0x04
MSG_NAV_STATUS = 0x14


class PointCloudWSNode(Node):
    """ROS2 node that serves point cloud data over WebSocket with bidirectional game commands."""

    def __init__(self):
        super().__init__('point_cloud_ws')

        self.declare_parameter('ws_port', 9090)
        self.declare_parameter('ws_host', '0.0.0.0')
        self.declare_parameter('full_refresh_interval', 30.0)

        self.ws_port = self.get_parameter('ws_port').value
        self.ws_host = self.get_parameter('ws_host').value
        self.full_refresh_interval = self.get_parameter('full_refresh_interval').value

        # State (thread-safe via lock)
        self.lock = threading.Lock()
        self.current_points = b''
        self.current_status = b''
        self.current_nav_status = b''
        self.previous_point_count = 0
        self.last_full_send_time = 0.0
        self._ws_clients = set()

        # Subscribers
        self.pointcloud_sub = self.create_subscription(
            PointCloud2, '/map_3d/points', self.pointcloud_callback, 10
        )
        self.status_sub = self.create_subscription(
            String, '/map_3d/status', self.status_callback, 10
        )
        # A2: Subscribe to nav status from nav2_goal_sender
        self.nav_status_sub = self.create_subscription(
            String, '/smolseek/nav_status', self.nav_status_callback, 10
        )

        # Fallback: subscribe to /scan (2D LiDAR) when VSLAM isn't tracking
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10
        )
        self.scan_points = b''  # accumulated scan-derived points
        self._has_vslam_data = False  # tracks whether /map_3d/points has real data

        # Fallback: subscribe to /map (occupancy grid from Cartographer)
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL)
        )
        self.map_points = b''  # points derived from occupancy grid

        # A2: Publisher for game nav goals → nav2_goal_sender
        self.goal_pub = self.create_publisher(
            String, '/smolseek/goal', 10
        )

        # Start WebSocket server in background thread
        self.ws_thread = threading.Thread(target=self.run_ws_server, daemon=True)
        self.ws_thread.start()

        self.get_logger().info(f'WebSocket server starting on ws://{self.ws_host}:{self.ws_port}')

    # ------------------------------------------------------------------
    # ROS2 callbacks
    # ------------------------------------------------------------------
    def pointcloud_callback(self, msg: PointCloud2):
        """Store latest point cloud data from VSLAM."""
        data = bytes(msg.data)
        with self.lock:
            if len(data) > 0:
                self._has_vslam_data = True
            self.current_points = data

    def status_callback(self, msg: String):
        """Store latest status JSON."""
        with self.lock:
            self.current_status = msg.data.encode('utf-8')

    def nav_status_callback(self, msg: String):
        """Store latest nav status JSON from nav2_goal_sender."""
        with self.lock:
            self.current_nav_status = msg.data.encode('utf-8')
        self.get_logger().info(f'Nav status update: {msg.data}')

    def scan_callback(self, msg: LaserScan):
        """Convert 2D LaserScan to XYZ points as fallback when VSLAM is down."""
        with self.lock:
            if self._has_vslam_data:
                return  # prefer VSLAM data when available

        points = []
        for i, r in enumerate(msg.ranges):
            if r < msg.range_min or r > msg.range_max or math.isinf(r) or math.isnan(r):
                continue
            angle = msg.angle_min + i * msg.angle_increment
            x = r * math.cos(angle)
            y = r * math.sin(angle)
            points.append((x, y, 0.0))

        if points:
            arr = np.array(points, dtype=np.float32)
            with self.lock:
                self.scan_points = arr.tobytes()
                self._rebuild_fallback_points()

    def _rebuild_fallback_points(self):
        """Merge map_points + scan_points into current_points (called with lock held)."""
        if self._has_vslam_data:
            return
        combined = self.map_points + self.scan_points
        if combined:
            self.current_points = combined

    def map_callback(self, msg: OccupancyGrid):
        """Convert 2D OccupancyGrid to XYZ points for 3D visualization."""
        resolution = msg.info.resolution
        origin_x = msg.info.origin.position.x
        origin_y = msg.info.origin.position.y
        width = msg.info.width

        points = []
        for i, cell in enumerate(msg.data):
            if cell > 50:  # occupied cells (walls)
                gx = (i % width) * resolution + origin_x
                gy = (i // width) * resolution + origin_y
                points.append((gx, gy, 0.0))
                # Add a second point slightly above for wall thickness
                points.append((gx, gy, 0.15))

        if points:
            arr = np.array(points, dtype=np.float32)
            with self.lock:
                self.map_points = arr.tobytes()
                self._rebuild_fallback_points()
                self.get_logger().info(
                    f'Map fallback: {len(points)} wall points from occupancy grid'
                )

    # ------------------------------------------------------------------
    # Frame building
    # ------------------------------------------------------------------
    def build_frame(self, msg_type: int, payload: bytes) -> bytes:
        """Build a binary WebSocket frame: [type_byte][payload]."""
        return struct.pack('B', msg_type) + payload

    # ------------------------------------------------------------------
    # WebSocket server
    # ------------------------------------------------------------------
    def run_ws_server(self):
        """Run the asyncio WebSocket server."""
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        loop.run_until_complete(self.ws_main())

    async def ws_main(self):
        """WebSocket server main loop."""
        if websockets is None:
            self.get_logger().error('websockets package not installed. Run: pip3 install websockets')
            return

        async with serve(self.ws_handler, self.ws_host, self.ws_port):
            self.get_logger().info(f'WebSocket server listening on ws://{self.ws_host}:{self.ws_port}')
            await asyncio.Future()  # Run forever

    async def ws_handler(self, websocket):
        """Handle a single WebSocket client — send and recv run concurrently."""
        self._ws_clients.add(websocket)
        client_addr = websocket.remote_address
        self.get_logger().info(f'Client connected: {client_addr}')

        try:
            # Send initial full cloud + status
            await self.send_full_cloud(websocket)
            await self.send_status(websocket)

            # Run send loop and receive loop concurrently
            await asyncio.gather(
                self._send_loop(websocket),
                self._recv_loop(websocket),
            )
        except websockets.exceptions.ConnectionClosed:
            self.get_logger().info(f'Client disconnected: {client_addr}')
        finally:
            self._ws_clients.discard(websocket)

    async def _send_loop(self, websocket):
        """Push point cloud frames, status, and nav_status to a WS client."""
        last_full_send = time.time()
        last_nav_status = b''
        last_status = b''

        while True:
            # Faster loop when in fallback mode (live LiDAR at 10Hz)
            with self.lock:
                in_fallback = not self._has_vslam_data
            await asyncio.sleep(0.25 if in_fallback else 1.0)
            now = time.time()

            # Send map status only when changed
            with self.lock:
                status = self.current_status
            if status and status != last_status:
                last_status = status
                frame = self.build_frame(MSG_STATUS, status)
                await websocket.send(frame)

            # Forward nav_status if changed (0x14 header + JSON)
            with self.lock:
                nav_status = self.current_nav_status
            if nav_status and nav_status != last_nav_status:
                last_nav_status = nav_status
                frame = self.build_frame(MSG_NAV_STATUS, nav_status)
                await websocket.send(frame)

            # In fallback mode: send full cloud every cycle (scan data changes each frame)
            # In VSLAM mode: full refresh every 30s, deltas otherwise
            if in_fallback:
                await self.send_full_cloud(websocket)
            elif now - last_full_send >= self.full_refresh_interval:
                await self.send_full_cloud(websocket)
                last_full_send = now
            else:
                await self.send_delta(websocket)

    async def _recv_loop(self, websocket):
        """Receive text messages from WS client and dispatch to ROS2."""
        async for raw_msg in websocket:
            # Only handle text (string) messages
            if not isinstance(raw_msg, str):
                self.get_logger().debug('Ignoring binary WS message from client')
                continue

            try:
                data = json.loads(raw_msg)
            except json.JSONDecodeError as e:
                self.get_logger().warn(f'Bad JSON from WS client: {e}')
                continue

            msg_type = data.get('type', '')
            self.get_logger().info(f'WS recv — type={msg_type} payload={raw_msg}')

            if msg_type == 'nav_goal':
                self._handle_nav_goal(data)
            else:
                self.get_logger().warn(f'Unknown WS message type: {msg_type}')

    def _handle_nav_goal(self, data: dict):
        """Republish a nav_goal from WS to /smolseek/goal ROS2 topic."""
        goal_json = json.dumps({
            'x': float(data.get('x', 0.0)),
            'y': float(data.get('y', 0.0)),
            'theta': float(data.get('theta', 0.0)),
            'cache_id': str(data.get('cache_id', '')),
        })
        msg = String()
        msg.data = goal_json
        self.goal_pub.publish(msg)
        self.get_logger().info(f'Published /smolseek/goal: {goal_json}')

    # ------------------------------------------------------------------
    # Point cloud send helpers
    # ------------------------------------------------------------------
    async def send_full_cloud(self, websocket):
        """Send full point cloud to a client. Falls back to scan/map data."""
        with self.lock:
            data = self.current_points
            # Fallback chain: VSLAM points → map points → live scan
            if not data:
                data = self.map_points
            if not data:
                data = self.scan_points
            self.previous_point_count = len(data) // 12  # 12 bytes per point (3 * float32)

        if data:
            frame = self.build_frame(MSG_FULL_CLOUD, data)
            await websocket.send(frame)

    async def send_delta(self, websocket):
        """Send only new points since last full send."""
        with self.lock:
            data = self.current_points
            current_count = len(data) // 12

        if current_count > self.previous_point_count:
            offset = self.previous_point_count * 12
            delta_data = data[offset:]
            frame = self.build_frame(MSG_DELTA_POINTS, delta_data)
            await websocket.send(frame)
            self.previous_point_count = current_count

    async def send_status(self, websocket):
        """Send status JSON to a client."""
        with self.lock:
            data = self.current_status

        if data:
            frame = self.build_frame(MSG_STATUS, data)
            await websocket.send(frame)


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudWSNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
