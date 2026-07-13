# JetCarEdge

JetCarEdge is the Jetson-side data bridge for the JetCar project. It runs as a
ROS2 Python package, subscribes to camera/lidar/IMU topics, compresses camera
frames, uploads them to the cloud inference service by WebSocket, and publishes
AI results back into ROS2 for local safety handling.

## Technology Choice

- Runtime: Ubuntu on Jetson with ROS2 Foxy or Humble.
- Language: Python 3, because ROS2 Python nodes are fast to iterate and match
  the image/sensor upload workflow.
- ROS2 libraries: `rclpy`, `sensor_msgs`, `std_msgs`, `cv_bridge`.
- Network: `websocket-client`, using a persistent WebSocket connection to the
  cloud service.
- Image processing: OpenCV JPEG resize/compression before upload.

This repository intentionally does not replace the car's existing TCP remote
control service. The Flutter app can keep controlling the car over TCP while
this node only handles camera/sensor upload and AI result feedback.

## Repository Layout

```text
JetCarEdge/
  jetcar_edge/
    edge_upload_node.py     ROS2 node entrypoint
    image_codec.py          ROS Image -> JPEG base64 conversion
    models.py               Message schema helpers
    safety.py               Local danger decision helper
    sensor_buffer.py        Latest lidar/IMU cache
    ws_client.py            Reconnecting WebSocket worker
    motion_controller.py    Similarity visual-servo target alignment/approach
    cloud_discovery.py      Optional UDP Cloud IP discovery
  config/
    edge.yaml               Runtime configuration
  resource/
    jetcar_edge             ROS2 ament marker
  package.xml               ROS2 package metadata
  setup.py                  ROS2 Python package setup
  requirements.txt          Python-only dependencies
```

## Environment Commands To Run

Run these on the Jetson after copying this folder into a ROS2 workspace, for
example `~/yahboomcar_ws/src/JetCarEdge`.

```bash
cd ~/yahboomcar_ws/src
cp -r /path/to/JetCarEdge .

python3 -m pip install -r JetCarEdge/requirements.txt

cd ~/yahboomcar_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select jetcar_edge
source install/setup.bash
```

## Mock Camera Upload

When the real car/camera is not available, upload one local image as the cloud
reference frame:

```bash
cd /path/to/JetCarEdge
python scripts/upload_mock_camera.py \
  --cloud http://192.168.137.1:8000 \
  --car-id car_001 \
  --image ../yolov5-7.0/data/images/bus.jpg
```

After this succeeds, the mobile app can upload another image to compare against
this simulated camera frame.

## Mock Camera Server

For the newer request-response flow, run a tiny HTTP server on the edge side.
The cloud will request the current frame only when the phone uploads a query
image:

```bash
cd /path/to/JetCarEdge
python scripts/mock_camera_server.py \
  --host 0.0.0.0 \
  --port 8100 \
  --image ../yolov5-7.0/data/images/bus.jpg
```

Then configure JetCarCloud:

```bash
EDGE_FRAME_URL=http://127.0.0.1:8100/api/frame
```

Later, this server can keep the same `/api/frame` interface and replace the
fixed file read with a camera/video-frame capture.

Start the node after the cloud service is already listening:

```bash
ros2 run jetcar_edge edge_upload_node \
  --ros-args \
  -p car_id:=car_001 \
  -p stream_id:=camera_front \
  -p cloud_host:=192.168.137.1 \
  -p cloud_port:=8000 \
  -p algorithm_ids:="[yolov5-manhole-detect,yolov8-road-damage]" \
  -p camera_topic:=/camera/image_raw \
  -p scan_topic:=/scan \
  -p imu_topic:=/imu/data
```

For the current `jetcar_auto` container workflow, start camera and Edge manually
inside the container. Keep `docker_orchestrator_enabled=false` because this node
is already running inside the container and should not call `docker exec` on
itself:

```bash
docker start jetcar_auto
docker exec -it jetcar_auto bash

export ROS_DOMAIN_ID=30
source /opt/ros/foxy/setup.bash
cd /workspace
source install/setup.bash
```

Terminal A, camera:

```bash
export ROS_DOMAIN_ID=30
source /opt/ros/foxy/setup.bash
ros2 launch astra_camera astro_pro_plus.launch.xml enable_color:=true enable_depth:=false
```

Terminal B, Edge:

```bash
export ROS_DOMAIN_ID=30
source /opt/ros/foxy/setup.bash
cd /workspace
source install/setup.bash
ros2 run jetcar_edge edge_upload_node --ros-args \
  -p car_id:=car_001 \
  -p stream_id:=camera_front \
  -p cloud_url:=ws://192.168.137.126:8000/ws/video/car_001/camera_front/edge \
  -p camera_topic:=/camera/color/image_raw \
  -p algorithm_ids:="" \
  -p frame_server_port:=6000 \
  -p docker_orchestrator_enabled:=false
```

`cloud_url` may omit `algorithm_ids`; the node rewrites the query string when
the phone changes modes. For similarity, the phone/Edge control port will switch
the upload URL to `algorithm_ids=yolov5-similarity`.

The Edge node now also serves the latest camera frame at:

```text
GET http://<edge-ip>:6000/api/frame
GET http://<edge-ip>:6000/frame.jpg
```

This replaces the old separate `scripts/mock_camera_server.py --port 6000`
process for real camera runs. The HTTP frame server caches camera frames even
when AI upload is off, but Cloud upload still starts only after the phone sends
a non-empty algorithm list.

The node builds this Cloud upload URL automatically:

```text
ws://<cloud_host>:<cloud_port>/ws/video/<car_id>/<stream_id>/edge?algorithm_ids=<ids>&include_image=true
```

Set `cloud_url` only when you need to override the generated URL.

If Cloud is configured to broadcast a local discovery beacon, Edge can discover
the Cloud IP at startup:

```bash
ros2 run jetcar_edge edge_upload_node \
  --ros-args \
  -p cloud_discovery_enabled:=true \
  -p cloud_discovery_port:=8765
```

If no beacon is received within `cloud_discovery_listen_seconds`, Edge falls
back to the configured `cloud_host`.

Useful control topics:

```bash
ros2 topic pub /jetcar/ai_enable std_msgs/msg/Bool "{data: true}" --once
ros2 topic pub /jetcar/snapshot std_msgs/msg/Empty "{}" --once
ros2 topic pub /jetcar/algorithm_ids std_msgs/msg/String "{data: 'yolov5-manhole-detect,yolov8-road-damage'}" --once
ros2 topic echo /jetcar/ai_result
ros2 topic echo /jetcar/emergency_stop
```

`/jetcar/algorithm_ids` can also receive JSON:

```bash
ros2 topic pub /jetcar/algorithm_ids std_msgs/msg/String \
  "{data: '{\"algorithm_ids\":[\"yolov8-road-damage\"]}'}" --once
```

The node also exposes a phone-facing AI control TCP port, default `6001`. The
Flutter app sends one JSON object per line to this port:

```json
{"type":"jetcar_ai_control","mode":"road_inspection","mask":"TF","car_id":"car_001","stream_id":"camera_front"}
```

`TF`, `FT`, and `TT` enable manhole, road-damage, or both algorithms. `FF` or an
empty `algorithm_ids` list disables upload and disconnects the cloud WebSocket.
Similarity search uses:

```json
{"type":"jetcar_ai_control","mode":"similarity","algorithm_ids":["yolov5-similarity"],"car_id":"car_001","stream_id":"camera_front"}
```

Manual similarity control test from inside the container:

```bash
printf '{"type":"jetcar_ai_control","mode":"similarity","car_id":"car_001","stream_id":"camera_front","algorithm_ids":["yolov5-similarity"]}\n' | nc 127.0.0.1 6001
```

Stop AI upload:

```bash
printf '{"type":"jetcar_ai_control","mode":"off","car_id":"car_001","stream_id":"camera_front","algorithm_ids":[]}\n' | nc 127.0.0.1 6001
```

For automatic similarity search, Edge can call the existing Docker/ROS programs
from the Jetson operation manual. Fill the real container name or ID prefix in
`autodrive_container`, then enable the orchestrator:

```yaml
docker_orchestrator_enabled: true
autodrive_container: "2169"
docker_command_prefix: "source /opt/ros/foxy/setup.bash"
similarity_search_programs:
  - ros2 run yahboomcar_bringup Mcnamu_driver_X3
  - ros2 launch sllidar_ros2 sllidar_launch.py
  - ros2 launch astra_camera astra.launch.xml
```

When the phone starts `mode=similarity`, Edge runs:

```text
docker start <autodrive_container>
docker exec -d <autodrive_container> bash -lc "<program>"
```

This intentionally reuses the manual's existing chassis, lidar avoidance, and
camera programs instead of reimplementing the device bring-up path. For the
similarity target itself, Edge publishes `/cmd_vel` from Cloud's `center_norm`
result: first rotate to center the target, then approach slowly, and stop when
the front lidar distance reaches `similarity_target_stop_distance_m` or the
safety distance is crossed. Keep `docker_orchestrator_enabled=false` while
debugging the camera and Cloud path only.

The default `similarity_search_programs` intentionally starts chassis, lidar,
and camera only. The manual's `colorHSV` and `colorTracker` are HSV/color
tracking demos; they cannot directly track an arbitrary uploaded phone image
unless their source is changed to consume Cloud's feature/center result. For the
uploaded-image similarity target, Edge uses Cloud's `center_norm` and publishes
`/cmd_vel` itself.

The manual's autonomous lidar avoidance node can be useful, but do not enable it
until you know whether it also publishes `/cmd_vel`. If it does, it may conflict
with the visual-servo controller:

```yaml
  - ros2 run yahboomcar_laser laser_Avoidance_a1_X3
```

Avoid running another node that publishes conflicting `/cmd_vel` commands unless
you have verified its arbitration behavior on the real car.

### Debug Docker Commands On The Car

The commands in `edge.yaml` are likely to need real-car verification because
Yahboom images often differ in ROS distro, workspace path, package names, and
topic names. Use this sequence on the Jetson:

```bash
docker ps -a
docker start <container_id_or_name>
docker exec -it <container_id_or_name> bash
```

Inside the container:

```bash
printenv ROS_DISTRO
ls /opt/ros
find / -maxdepth 4 -name setup.bash 2>/dev/null
source /opt/ros/foxy/setup.bash
ros2 pkg list | grep -E 'icar|astra|sllidar|yahboom'
ros2 node list
ros2 topic list
ros2 topic info /cmd_vel
```

Then validate each candidate command one by one in the foreground:

```bash
ros2 run yahboomcar_bringup Mcnamu_driver_X3
ros2 launch sllidar_ros2 sllidar_launch.py
ros2 launch astra_camera astra.launch.xml
```

Your `jetcar_auto` container reports `yahboomcar_*` packages, not `icar_*`
packages, so `ros2 run icar_bringup Mcnamu_driver_X3` is expected to fail with
`Package 'icar_bringup' not found`. If `/cmd_vel` is unknown before the base
driver starts, start the driver first and check topics again:

```bash
ros2 run yahboomcar_bringup Mcnamu_driver_X3
ros2 topic list
ros2 topic list | grep -E 'cmd|vel|velocity|joy|car'
ros2 topic info /cmd_vel
```

If the real velocity topic is not `/cmd_vel`, update `cmd_vel_topic` in
`config/edge.yaml`.

To find the original tracking source inside the container, use:

```bash
ros2 pkg prefix yahboomcar_astra
ros2 pkg prefix yahboomcar_laser
ros2 pkg executables yahboomcar_astra
ros2 pkg executables yahboomcar_laser
find /workspace /install /root -maxdepth 6 -type f \( -name '*Tracker*' -o -name '*tracker*' -o -name '*HSV*' -o -name '*.py' -o -name '*.cpp' \) 2>/dev/null
find / -path '*yahboomcar_astra*' -o -path '*yahboomcar_laser*' 2>/dev/null
```

If `ros2 pkg prefix` points under `/install`, that may be installed artifacts
only. Prefer editing the matching source under `/workspace/src` if it exists,
then rebuild the workspace. If only `/install` exists, you may need the original
image source package or mount your own patched package into the container.

If a command only works after sourcing a workspace, put that source command into
`docker_command_prefix`, for example:

```yaml
docker_command_prefix: "source /opt/ros/foxy/setup.bash && source /root/yahboomcar_ws/install/setup.bash"
```

Only after the foreground commands work should you set:

```yaml
docker_orchestrator_enabled: true
autodrive_container: "<container_id_or_name>"
```

## Message Contract

The edge node sends each video frame to `/ws/video/{car_id}/{stream_id}/edge`:

```json
{
  "car_id": "car_001",
  "image": {
    "encoding": "jpeg",
    "width": 640,
    "height": 480,
    "data": "base64-jpeg"
  }
}
```

The cloud service publishes algorithm results to the app WebSocket
`/ws/inference/{car_id}/app`:

```json
{
  "type": "algorithm_result",
  "ok": true,
  "algorithm_id": "yolov8-road-damage",
  "car_id": "car_001",
  "stream_id": "camera_front",
  "runner": "local",
  "latency_ms": 18.5,
  "result": {
    "detection_count": 1,
    "detections": []
  },
  "annotated_image": null,
  "error": ""
}
```
