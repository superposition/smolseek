# Jetson File Manifest

**Host:** jetson@192.168.0.221
**Container:** `ugv_jetson_ros_humble` (nvidia/cuda:11.8.0-cudnn8-devel-ubuntu22.04)
**ROS Distro:** Humble
**Workspace:** `/home/ws/ugv_ws`

## Workspace Packages (`/home/ws/ugv_ws/src/`)

| Package | Type | Notes |
|---------|------|-------|
| `smolROS/` | Our code | VSLAM mapping, ugv_map_bridge, config, scripts |
| `stella_vslam_ros/` | External | Stella VSLAM ROS2 wrapper |
| `ugv_else/` | Vendor | Cartographer config, misc UGV utilities |
| `ugv_main/` | Vendor | Core UGV driver, bringup |
| `rosclaw_discovery` | Symlink → `/home/jetson/ugv_ws/rosclaw/ros2_ws/src/rosclaw_discovery` |
| `rosclaw_msgs` | Symlink → `/home/jetson/ugv_ws/rosclaw/ros2_ws/src/rosclaw_msgs` |

## smolROS File Tree

```
smolROS/
├── config/
│   ├── explore_lite.yaml          # Frontier exploration params
│   ├── nav2_exploration.yaml      # Nav2 stack params
│   ├── nullclaw.json              # Claw config
│   ├── tum_mono_test.yaml         # TUM mono benchmark config
│   └── ugv_monocam.yaml           # Stella VSLAM camera config
├── scripts/
│   ├── install_crontab.sh
│   ├── prepare_mapping.sh
│   ├── publish_test_images.py
│   ├── serve_web.sh
│   ├── start_base_stack.sh
│   ├── start_rosbridge.sh
│   ├── tum_mono_test.yaml
│   ├── verify_tf_tree.sh
│   ├── vslam_start.sh
│   ├── vslam_status.sh
│   └── vslam_stop.sh
├── src/ugv_map_bridge/            # ROS2 ament_python package
│   ├── launch/
│   │   └── autonomous_mapping.launch.py
│   ├── scripts/
│   │   └── vslam_mcp_server.py
│   ├── ugv_map_bridge/
│   │   ├── __init__.py
│   │   ├── map_bridge_node.py     # VSLAM → PointCloud2 bridge
│   │   ├── msgpack_extractor.py   # Extract landmarks from .msg DB
│   │   └── point_cloud_ws.py      # WS server for 3D viewer + game cmds
│   ├── package.xml
│   ├── setup.cfg
│   └── setup.py
├── web/
│   ├── index.html                 # Babylon.js 3D viewer
│   └── viewer.js
├── PLAN.md
├── README.md
└── STATE.md
```

## Active ROS2 Topics

| Topic | Type | Source |
|-------|------|--------|
| `/scan` | sensor_msgs/LaserScan | LiDAR (LD19) |
| `/odom` | nav_msgs/Odometry | UGV driver |
| `/odom_rf2o` | nav_msgs/Odometry | rf2o laser odometry |
| `/imu/data` | sensor_msgs/Imu | IMU |
| `/image_raw/compressed` | sensor_msgs/CompressedImage | Camera |
| `/cmd_vel` | geometry_msgs/Twist | Velocity commands |
| `/tf`, `/tf_static` | tf2_msgs | Transform tree |
| `/voltage` | std_msgs/Float32 | Battery voltage |
| `/ugv/led_ctrl` | | LED control |
| `/ugv/joint_states` | sensor_msgs/JointState | Joint states |

## Game Topics (Track A)

| Topic | Type | Direction | Purpose |
|-------|------|-----------|---------|
| `/smolseek/goal` | std_msgs/String (JSON) | WS → ROS2 | Nav goal from game coordinator |
| `/smolseek/nav_status` | std_msgs/String (JSON) | ROS2 → WS | Navigation feedback to game |

## Entry Points (ugv_map_bridge)

| Executable | Module | Description |
|-----------|--------|-------------|
| `map_bridge_node` | `ugv_map_bridge.map_bridge_node:main` | VSLAM DB → PointCloud2 |
| `point_cloud_ws` | `ugv_map_bridge.point_cloud_ws:main` | WS server (bidirectional) |
| `nav2_goal_sender` | `ugv_map_bridge.nav2_goal_sender:main` | Nav2 goal dispatch |

## Connection Info

```bash
ssh jetson@192.168.0.221
docker exec -it ugv_jetson_ros_humble bash
source /opt/ros/humble/setup.bash
source /home/ws/ugv_ws/install/setup.bash
```
