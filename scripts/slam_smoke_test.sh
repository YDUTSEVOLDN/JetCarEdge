#!/usr/bin/env bash
set -e

CAR_IP=${CAR_IP:-192.168.137.239}
CONTAINER=${CONTAINER:-ros_x3_fixed}
SLAM_API_PORT=${SLAM_API_PORT:-8000}
ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-30}
ROS_GOAL_TOPIC=${ROS_GOAL_TOPIC:-/goal_pose}

echo "GET http://$CAR_IP:$SLAM_API_PORT/api/slam/map"
curl -sS "http://$CAR_IP:$SLAM_API_PORT/api/slam/map" | head -c 400 || true
echo

echo "POST http://$CAR_IP:$SLAM_API_PORT/api/slam/goal"
curl -sS -X POST "http://$CAR_IP:$SLAM_API_PORT/api/slam/goal" \
  -H "Content-Type: application/json" \
  -d '{"x":0.0,"y":0.0,"yaw":0.0,"frame_id":"map"}' | head -c 400 || true
echo

if command -v docker >/dev/null 2>&1; then
  if docker ps -a --format '{{.Names}}' | grep -qx "$CONTAINER"; then
    echo "ros2 topic echo $ROS_GOAL_TOPIC --once"
    docker exec -it "$CONTAINER" bash -lc "export ROS_DOMAIN_ID=$ROS_DOMAIN_ID; ros2 topic echo $ROS_GOAL_TOPIC --once" || true
    exit 0
  fi
fi

echo "ros2 topic echo $ROS_GOAL_TOPIC --once"
export ROS_DOMAIN_ID="$ROS_DOMAIN_ID"
ros2 topic echo "$ROS_GOAL_TOPIC" --once || true

