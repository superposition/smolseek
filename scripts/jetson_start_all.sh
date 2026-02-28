#!/usr/bin/env bash
# jetson_start_all.sh — Start all smolseek ROS2 nodes + services on the Jetson.
# Designed to be idempotent: safe to re-run. Kills stale processes first.
#
# Services (10):
#   HOST:
#     1. Web app static server          :8080
#     2. OpenClaw gateway (systemd)      :3000
#   DOCKER (ugv_jetson_ros_humble):
#     3. Rosbridge                       :9091  (for OpenClaw + CameraFeed)
#     4. Robot bringup                          (ugv_driver, LiDAR LD19, rf2o laser odom, URDF TFs)
#     5. Cartographer SLAM                      (real 2D mapping → /map)
#     6. USB Camera                             (YUYV 640x480@30fps → /camera/image_raw)
#     7. Camera TF bridge                       (pt_camera_link → camera)
#     8. Stella VSLAM                           (monocular visual SLAM)
#     9. Map bridge node                        (VSLAM landmarks → /map_3d/points)
#    10. Point cloud WS server           :9090  (binary stream to browser)
#
# Usage:  ssh jetson@192.168.0.221 "bash /home/jetson/jetson_start_all.sh"

set -euo pipefail

CONTAINER="ugv_jetson_ros_humble"
WEB_DIR="/home/jetson/smolseek-web"

log() { echo "[smolseek] $(date '+%H:%M:%S') $*"; }

# --- Helper: run a command inside the Docker container in background ---
# docker exec -d returns immediately; inner command must redirect all I/O.
dexec() {
  docker exec -d "$CONTAINER" bash -c "
    export UGV_MODEL=ugv_rover
    export LDLIDAR_MODEL=ld19
    source /opt/ros/humble/setup.bash
    source /home/ws/ugv_ws/install/setup.bash
    exec $1
  "
}

# --- Helper: kill a process by name inside the container ---
dkill() {
  docker exec "$CONTAINER" bash -c "pkill -f '$1' 2>/dev/null; exit 0" || true
  sleep 0.5
}

# ============================================================
# 1. Web app static server (runs on HOST, port 8080)
# ============================================================
log "1/10  Web server on :8080"
kill $(lsof -ti :8080) 2>/dev/null || true
cd "$WEB_DIR" && nohup python3 -m http.server 8080 --bind 0.0.0.0 > /tmp/smolseek-web.log 2>&1 &

# ============================================================
# 2. Rosbridge (for OpenClaw + CameraFeed, port 9091)
# ============================================================
log "2/10  Rosbridge on :9091"
dkill "rosbridge_websocket"
dexec "ros2 launch rosbridge_server rosbridge_websocket_launch.xml port:=9091 > /tmp/rosbridge.log 2>&1"
sleep 2

# ============================================================
# 3. Robot bringup (driver + LiDAR + laser odom + URDF TFs)
# ============================================================
log "3/10  Robot bringup (ugv_rover + ld19)"
dkill "ugv_bringup"
dkill "ugv_driver"
dkill "ldlidar_node"
dkill "rf2o_laser_odometry"
dkill "robot_state_publisher"
dkill "base_node"
dexec "ros2 launch ugv_bringup bringup_lidar.launch.py pub_odom_tf:=true > /tmp/bringup_lidar.log 2>&1"
sleep 5

# ============================================================
# 4. Cartographer SLAM (real 2D mapping from LiDAR + odom)
# ============================================================
log "4/10  Cartographer mapping"
dkill "cartographer"
dexec "ros2 launch cartographer mapping.launch.py > /tmp/cartographer.log 2>&1"
sleep 2

# ============================================================
# 5. USB Camera (YUYV 640x480 @ 30fps)
# ============================================================
log "5/10  USB camera"
dkill "usb_cam_node"
dexec "ros2 run usb_cam usb_cam_node_exe --ros-args \
  -p video_device:=/dev/video0 \
  -p image_width:=640 \
  -p image_height:=480 \
  -p framerate:=30.0 \
  -p pixel_format:=yuyv \
  -r /image_raw:=/camera/image_raw \
  > /tmp/usb_cam.log 2>&1"
sleep 1

# ============================================================
# 6. Static TF: camera frame bridge
# ============================================================
log "6/10  Camera TF (pt_camera_link → camera)"
dkill "static_transform_publisher.*pt_camera_link"
dexec "ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 pt_camera_link camera > /tmp/cam_tf.log 2>&1"

# ============================================================
# 7. Stella VSLAM
# ============================================================
log "7/10  Stella VSLAM"
dkill "run_slam"
dexec "ros2 run stella_vslam_ros run_slam \
  -v /home/ws/ugv_ws/vocab/orb_vocab.fbow \
  -c /home/ws/ugv_ws/src/smolROS/config/ugv_monocam.yaml \
  -o /home/ws/ugv_ws/maps/current_map.msg \
  --viewer none \
  --ros-args -p publish_tf:=true -p odom2d:=false \
  > /tmp/stella_vslam.log 2>&1"

# ============================================================
# 8. Map bridge node (VSLAM → PointCloud2)
# ============================================================
log "8/10  Map bridge"
dkill "map_bridge_node"
dexec "ros2 run ugv_map_bridge map_bridge_node --ros-args \
  -p map_db_path:=/home/ws/ugv_ws/maps/current_map.msg \
  -p extract_interval:=5.0 \
  -p frame_id:=map \
  > /tmp/map_bridge.log 2>&1"

# ============================================================
# 9. Point cloud WebSocket server (port 9090)
# ============================================================
log "9/10  Point cloud WS on :9090"
dkill "point_cloud_ws"
dexec "ros2 run ugv_map_bridge point_cloud_ws --ros-args \
  -p ws_port:=9090 \
  -p ws_host:=0.0.0.0 \
  > /tmp/ws.log 2>&1"

# ============================================================
# 10. Restart OpenClaw gateway (host systemd)
# ============================================================
log "10/10 OpenClaw gateway"
systemctl --user restart openclaw-gateway 2>/dev/null || true

# ============================================================
# Health check
# ============================================================
sleep 3
log "Checking status..."

docker exec "$CONTAINER" bash -c '
  source /opt/ros/humble/setup.bash && \
  source /home/ws/ugv_ws/install/setup.bash && \
  echo "=== ROS2 Nodes ===" && ros2 node list 2>/dev/null && \
  echo "" && echo "=== Key Topics ===" && ros2 topic list 2>/dev/null | grep -E "scan|map|odom|camera|smolseek|cmd_vel"
'

log ""
log "Endpoints:"
log "  Web app:      http://192.168.0.221:8080/?ws_port=9090"
log "  Spectator:    http://192.168.0.221:8080/?ws_port=9090&mode=spectator"
log "  OpenClaw:     http://192.168.0.221:3000"
log "  WS stream:    ws://192.168.0.221:9090"
log "  Rosbridge:    ws://192.168.0.221:9091"
log "Done."
