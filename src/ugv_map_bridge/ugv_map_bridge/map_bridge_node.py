"""ROS2 node bridging stella_vslam output to a PointCloud2 topic and status topic.

Subscribes to stella_vslam pose and keyframe topics, periodically reads the
map database to extract 3D landmarks, and publishes them as PointCloud2
messages for downstream consumers (WebSocket server, RViz, etc.).
"""

import json
import struct
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header, String

from ugv_map_bridge.msgpack_extractor import extract_landmarks, extract_keyframes


class MapBridgeNode(Node):
    """Bridge between stella_vslam and downstream consumers."""

    def __init__(self):
        super().__init__('map_bridge_node')

        # Parameters
        self.declare_parameter('map_db_path', '/home/jetson/ugv_ws/maps/current_map.msg')
        self.declare_parameter('extract_interval', 5.0)  # seconds between map reads
        self.declare_parameter('frame_id', 'map')

        self.map_db_path = self.get_parameter('map_db_path').value
        self.extract_interval = self.get_parameter('extract_interval').value
        self.frame_id = self.get_parameter('frame_id').value

        # State
        self.current_pose = None
        self.total_points = 0
        self.total_keyframes = 0
        self.tracking_state = 'initializing'
        self.start_time = time.time()
        self.last_points = np.empty((0, 3), dtype=np.float32)

        # QoS for stella_vslam topics (best effort, volatile)
        vslam_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=1,
        )

        # Subscribe to stella_vslam camera pose (Odometry from /run_slam/camera_pose)
        self.pose_sub = self.create_subscription(
            Odometry, '/run_slam/camera_pose', self.pose_callback, vslam_qos
        )

        # Publishers
        self.pointcloud_pub = self.create_publisher(PointCloud2, '/map_3d/points', 10)
        self.status_pub = self.create_publisher(String, '/map_3d/status', 10)

        # Timer for periodic map extraction
        self.extract_timer = self.create_timer(self.extract_interval, self.extract_and_publish)

        # Timer for status publishing (every 2s)
        self.status_timer = self.create_timer(2.0, self.publish_status)

        self.get_logger().info(
            f'Map bridge node started. Reading map from: {self.map_db_path}'
        )

    def pose_callback(self, msg: Odometry):
        """Handle camera pose updates from stella_vslam."""
        self.current_pose = msg.pose.pose
        self.tracking_state = 'tracking'

    def extract_and_publish(self):
        """Read the map database and publish updated point cloud."""
        try:
            points = extract_landmarks(self.map_db_path)
            keyframes = extract_keyframes(self.map_db_path)
        except Exception as e:
            self.get_logger().warn(f'Failed to extract map data: {e}')
            return

        self.total_points = len(points)
        self.total_keyframes = len(keyframes)

        if self.total_points == 0:
            return

        # Publish full point cloud
        pc2_msg = self.numpy_to_pointcloud2(points)
        self.pointcloud_pub.publish(pc2_msg)

        # Track for delta computation
        self.last_points = points

        self.get_logger().info(
            f'Published {self.total_points} points, {self.total_keyframes} keyframes'
        )

    def numpy_to_pointcloud2(self, points: np.ndarray) -> PointCloud2:
        """Convert Nx3 numpy array to PointCloud2 message."""
        msg = PointCloud2()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        msg.height = 1
        msg.width = len(points)

        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]

        msg.is_bigendian = False
        msg.point_step = 12  # 3 * float32
        msg.row_step = msg.point_step * msg.width
        msg.data = points.astype(np.float32).tobytes()
        msg.is_dense = True

        return msg

    def publish_status(self):
        """Publish status JSON for WebSocket server and NullClaw."""
        elapsed = time.time() - self.start_time
        status = {
            'total_points': self.total_points,
            'total_keyframes': self.total_keyframes,
            'tracking_state': self.tracking_state,
            'elapsed_seconds': int(elapsed),
            'has_pose': self.current_pose is not None,
        }

        if self.current_pose is not None:
            p = self.current_pose.position
            status['robot_position'] = {'x': p.x, 'y': p.y, 'z': p.z}

        msg = String()
        msg.data = json.dumps(status)
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MapBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
