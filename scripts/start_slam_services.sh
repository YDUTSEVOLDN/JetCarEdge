#!/usr/bin/env bash
set -e

CAR_IP=${CAR_IP:-192.168.137.239}
CONTAINER=${CONTAINER:-ros_x3_fixed}
ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-30}
ROS_GOAL_TOPIC=${ROS_GOAL_TOPIC:-/goal_pose}
SLAM_API_HOST=${SLAM_API_HOST:-0.0.0.0}
SLAM_API_PORT=${SLAM_API_PORT:-8000}
JETCAR_WS=${JETCAR_WS:-/workspace}

start_inside() {
  export ROS_DOMAIN_ID="$ROS_DOMAIN_ID"
  export ROS_GOAL_TOPIC="$ROS_GOAL_TOPIC"
  export SLAM_API_HOST="$SLAM_API_HOST"
  export SLAM_API_PORT="$SLAM_API_PORT"
  if [ -f "/opt/ros/foxy/setup.bash" ]; then
    . /opt/ros/foxy/setup.bash
  fi
  if [ -f "/opt/ros/humble/setup.bash" ]; then
    . /opt/ros/humble/setup.bash
  fi
  if [ -f "$JETCAR_WS/install/setup.bash" ]; then
    . "$JETCAR_WS/install/setup.bash"
  fi
  nohup ros2 run jetcar_edge slam_web_api >/tmp/jetcar_slam_api.log 2>&1 &
  echo "slam_web_api started pid=$! host=$SLAM_API_HOST port=$SLAM_API_PORT"
}

if command -v docker >/dev/null 2>&1; then
  docker ps >/dev/null 2>&1 || true
  if docker ps -a --format '{{.Names}}' | grep -qx "$CONTAINER"; then
    docker start "$CONTAINER" >/dev/null
    docker exec -d "$CONTAINER" bash -lc "$(declare -f start_inside); start_inside"
    echo "http://$CAR_IP:$SLAM_API_PORT/api/slam/map"
    exit 0
  fi
fi

start_inside
echo "http://$CAR_IP:$SLAM_API_PORT/api/slam/map"
