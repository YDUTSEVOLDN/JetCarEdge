# JetCar 实时视频流与 YOLO 推理全流程落地说明

本文基于当前三个仓库现状整理：

- `d:\CAR29\JetCar`：Flutter 移动端，当前已具备小车 TCP 控制，视频区域还是占位 UI。
- `d:\CAR29\JetCarEdge`：Jetson 侧 ROS2 边缘节点，当前已具备订阅 `/camera/image_raw`、压缩 JPEG、通过 WebSocket 上传云端、接收 AI 结果回灌 ROS2 的能力。
- `d:\CAR29\JetCarCloud`：云端推理服务，当前已具备接收 JPEG 图像、解码、YOLO 推理、结果广播给边缘端和 App 的能力。

目标：完整打通小车摄像头实时视频数据流，并优先推荐 `Jetson 本地 YOLO 推理` 的低延迟方案。

## 0. 整体数据流总览图

```text
推荐主链路（实时控制优先）

摄像头设备
└─ CSI/MIPI 或 USB 摄像头
   └─ Jetson 驱动层
      ├─ /dev/video0...                USB/UVC 常见
      ├─ Argus / nvarguscamerasrc      CSI 摄像头常见
      └─ 原生格式：NV12 / YUYV / MJPEG / H264 等
         └─ ROS2 采集节点
            ├─ v4l2_camera
            ├─ gscam / gstreamer 节点
            └─ 自定义 rclpy / rclcpp 采集节点
               └─ 发布 /camera/image_raw
                  └─ YOLO 推理节点（Jetson 本地）
                     ├─ sensor_msgs/Image -> cv::Mat / numpy
                     ├─ BGR/RGB resize + letterbox
                     ├─ TensorRT / Ultralytics / YOLOv5 推理
                     ├─ 绘制框 / 类别 / 置信度
                     ├─ 发布 /camera/detections
                     ├─ 发布 /camera/image_annotated
                     └─ 发布 /jetcar/emergency_stop 或危险事件
                        ├─ 小车本地 ROS2 节点订阅，做安全控制
                        ├─ JetCarEdge 网络分发节点
                        │  ├─ 上传检测结果到云端
                        │  ├─ 上传低码率预览流到移动端
                        │  └─ 上传事件与抓拍图到云端
                        ├─ Flutter / HarmonyOS 客户端
                        │  ├─ 看预览画面
                        │  └─ 看检测框/告警信息
                        └─ 云端 JetCarCloud
                           ├─ 存储事件
                           ├─ 远程展示
                           └─ 非实时复核 / 统计分析

备选链路（带宽充足但实时性较差）

摄像头 -> Jetson 采集节点 -> 压缩视频/JPEG -> 云端/手机端 -> 远端 YOLO 推理 -> 结果返回
```

## 1. 摄像头原生输出的数据格式、类型、尺寸、编码形式

### 1.1 当前仓库里已经能确认的信息

当前代码并没有直接管理摄像头驱动，只假设 ROS2 中已经存在 `/camera/image_raw`：

- `JetCarEdge` 默认订阅话题：`/camera/image_raw`
- 上传前会把 `sensor_msgs/Image` 转成 OpenCV `bgr8`
- 再缩放到目标宽度后编码成 JPEG + Base64 上传云端

这说明：

- ROS2 上游节点最终提供的是 `sensor_msgs/msg/Image`
- `JetCarEdge` 假设它能被 `cv_bridge` 正常转换成 `bgr8`
- 但“摄像头原生输出格式”目前**无法仅从仓库静态代码确定**

### 1.1.1 根据当前 Jetson 实机输出，已经确认的事实

你刚刚在 Jetson 上提供的信息，已经足够确认下面几点：

- 当前不是普通 USB `v4l2` 方案主导，而是 **Jetson CSI / Argus 相机链路**
- `gst-inspect-1.0 nvarguscamerasrc` 已正常返回，说明 `Argus` 摄像头源可用
- `nvarguscamerasrc` 的 `SRC template` 明确给出能力：
  - 格式：`video/x-raw(memory:NVMM)`
  - 像素格式：`NV12`
  - 内存类型：`NVMM`，即 Jetson/NVIDIA 显存侧缓冲
  - 帧率：理论上支持宽范围，实际受 sensor mode 限制
- 因此可以判定：**摄像头原生输出优先按 `NV12` 未压缩图像处理，而不是 MJPEG/H264 文件流**

当前还没有完全拿到的，是这三项：

1. 具体启用的 `width/height/framerate`
2. ROS2 `/camera/image_raw` 的最终 `encoding`
3. 实际运行时稳定帧率和是否掉帧

另外，`v4l2-ctl: command not found` 只说明 Jetson 上还没安装 `v4l-utils`，不代表没有摄像头：

```bash
sudo apt update
sudo apt install -y v4l-utils
```

但就今天的主目标来说，即便先不装 `v4l-utils`，你也已经可以继续走 `nvarguscamerasrc -> ROS2 -> YOLO` 这条主链路了。

### 1.1.2 当前 AstraPlus 图像话题的实机确认值

在 Docker 容器内启动相机后，使用：

```bash
timeout 3 ros2 topic echo /camera/color/image_raw
```

已经确认当前 RGB 图像话题参数如下：

- 话题名：`/camera/color/image_raw`
- `frame_id`：`camera_color_optical_frame`
- `height`：`480`
- `width`：`640`
- `encoding`：`rgb8`
- `is_bigendian`：`0`
- `step`：`1920`

这些值可以直接推导出：

- 当前给 YOLO 的是 **彩色图像**
- 每个像素 3 通道
- 每通道 8 bit
- 单行字节数 `1920 = 640 x 3`
- 所以当前 ROS2 图像可按 **640x480 RGB8** 理解

对当前工程意味着：

1. `JetCarEdge` 订阅话题不应再假设成默认 `/camera/image_raw`，而应优先改为：

```bash
/camera/color/image_raw
```

2. `cv_bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")` 依然可以正常使用  
   因为它会把 `rgb8` 自动转换成 OpenCV 常用的 `bgr8`

3. 这个分辨率已经很适合作为第一阶段 YOLO 输入  
   一开始不建议再上 720p/1080p，先把 `640x480` 实时链路跑通更稳

### 1.2 Jetson 上最常见的原生摄像头输出

#### 场景 A：CSI/MIPI 摄像头

Jetson 上常见走 `Argus` / `nvarguscamerasrc`，原生格式通常是：

- 数据格式：`NV12`
- 数据类型：`uint8`
- 尺寸：取决于传感器支持的 mode，比如 `1280x720`、`1920x1080`、`3264x2464`
- 编码形式：通常是未压缩 YUV 平面格式，不是 JPEG 文件流

#### 场景 B：USB UVC 摄像头

常见输出：

- `YUYV` / `YUY2`：未压缩 YUV422
- `MJPEG`：摄像头端已压缩 JPEG 帧流
- 少数支持 `H264`

### 1.3 在 Jetson 上如何拿到真实参数

先看设备：

```bash
v4l2-ctl --list-devices
ls /dev/video*
```

查看某个设备支持的格式、分辨率、帧率：

```bash
v4l2-ctl -d /dev/video0 --list-formats-ext
v4l2-ctl -d /dev/video0 --all
```

如果是 CSI 摄像头，建议再看 GStreamer 设备能力：

```bash
gst-device-monitor-1.0 Video/Source
gst-inspect-1.0 nvarguscamerasrc
```

如果你已经有 ROS2 图像话题，可以直接看 ROS2 层输出：

```bash
ros2 topic list | grep camera
ros2 topic info /camera/image_raw
ros2 topic hz /camera/image_raw
ros2 topic echo /camera/image_raw/header --once
```

如果你的环境是 `ROS2 Foxy`，有些发行版里的 `ros2 topic echo` 可能不支持 `--once`。  
这时改用下面两种方式之一：

```bash
timeout 3 ros2 topic echo /camera/image_raw
```

或者只看前几十行：

```bash
ros2 topic echo /camera/image_raw | head -n 40
```

更实用的是写一个最小订阅器打印图像元信息：

```python
def callback(msg):
    print("encoding=", msg.encoding)
    print("width=", msg.width, "height=", msg.height)
    print("step=", msg.step)
    print("is_bigendian=", msg.is_bigendian)
```

### 1.4 推荐你今天先做的确认动作

先在 Jetson 上拿到这四项：

1. 原生设备路径：`/dev/video0` 还是 CSI Argus
2. 支持分辨率和帧率：`640x480 / 1280x720 / 1920x1080`
3. 原生格式：`NV12 / YUYV / MJPEG`
4. ROS2 话题输出格式：`bgr8 / rgb8 / mono8 / nv12`

只有这四项确认后，后面采集链路才不会选错。

## 2. ROS2 下读取摄像头实时画面的可行方案

### 2.1 方案一：`v4l2_camera` 采集节点

适合：

- USB 摄像头
- 希望快速接 ROS2 标准图像话题

实现方式：

```bash
ros2 run v4l2_camera v4l2_camera_node --ros-args -p video_device:=/dev/video0
```

优点：

- 上手快
- 标准 ROS2 话题输出
- 调试成本低

缺点：

- 对 Jetson CSI 摄像头不一定最优
- 硬件加速能力一般
- 高分辨率高帧率下 CPU 压力可能偏大

### 2.2 方案二：`gscam` / GStreamer ROS2 节点

适合：

- Jetson CSI 摄像头
- 需要调用 `nvarguscamerasrc`、`nvv4l2decoder`、`nvvidconv`
- 追求低 CPU 占用和更稳帧率

典型 GStreamer 管道：

```bash
nvarguscamerasrc !
video/x-raw(memory:NVMM),width=1280,height=720,framerate=30/1,format=NV12 !
nvvidconv !
video/x-raw,format=BGRx !
videoconvert !
video/x-raw,format=BGR !
appsink
```

优点：

- 最贴合 Jetson
- 可利用硬件编解码与颜色空间转换
- 后续扩展到 RTSP/H264 也很顺

缺点：

- 管道参数多
- 初期配置复杂
- 格式协商出错时排查成本较高

### 2.3 方案三：自定义 ROS2 采集节点

实现方式：

- `rclpy + OpenCV VideoCapture`
- 或 `rclcpp + GStreamer / V4L2`

最简思路：

```python
cap = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)
ret, frame = cap.read()
msg = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
publisher.publish(msg)
```

优点：

- 控制力最强
- 可以把采集、预处理、限帧、时间戳统一管理
- 最适合和 YOLO 节点、上传节点一起联调

缺点：

- 需要自己处理异常重连、缓冲、同步、丢帧
- 代码维护量最高

### 2.4 结论

今天的工程落地建议：

- USB 摄像头：先用 `v4l2_camera` 跑通
- CSI 摄像头：优先用 `GStreamer + 自定义采集节点`

如果你目标是“今天就把 Jetson 本地 YOLO 跑起来”，推荐直接做：

`GStreamer 采集节点 -> /camera/image_raw -> 本地 YOLO 节点`

## 3. 原始图像数据后的两条处理路线

### 3.1 路线 A：Jetson 本地实时 YOLO 推理

链路：

`摄像头 -> ROS2 图像话题 -> Jetson YOLO -> 本地控制/上传结果`

适用场景：

- 巡检过程中需要实时避障、停车、告警
- 弱网、断网仍要工作
- 对延迟敏感

优点：

- 延迟最低
- 闭环最稳
- 网络只传结果或低码率预览，带宽压力小

缺点：

- Jetson 算力有限，需要控分辨率和模型大小
- 模型部署要做 TensorRT / half / INT8 优化

典型延迟：

- 采集 + 转换：`5~15 ms`
- YOLO 推理：`10~60 ms`，取决于模型和分辨率
- 本地总链路：常见 `30~100 ms`

### 3.2 路线 B：传到云端/手机端再推理

链路：

`摄像头 -> 压缩传输 -> 云端/手机 -> 远端 YOLO -> 结果回传`

适用场景：

- 非实时分析
- 云端算力更强，需要大模型
- 主要做远程复核、录像分析、地图标注

优点：

- 算力弹性大
- 便于集中管理模型
- 方便做多车统一分析

缺点：

- 网络抖动直接影响结果时效
- 上传视频流占带宽
- 控制闭环不可靠

典型延迟：

- JPEG 帧上传：`80~300+ ms`
- RTSP 到云端再解码推理：`200~800+ ms`
- 手机端推理：更不稳定，不建议做主控制链路

### 3.3 结论

主链路推荐：

- `Jetson 本地推理` 负责实时控制和危险检测
- `云端` 负责结果汇总、历史记录、远程看图
- `移动端` 负责看预览、看结果、发控制指令

## 4. 两种路线下的数据传输方式讲解

### 4.1 ROS2 话题本地分发

原理：

- 基于 DDS
- 节点之间发布/订阅消息
- 适合 Jetson 本机或局域网内 ROS2 节点通信

常用封装：

- `sensor_msgs/Image`
- `sensor_msgs/CompressedImage`
- 自定义 `DetectionArray`
- `std_msgs/String` 携带 JSON

实现思路：

- 采集节点发 `/camera/image_raw`
- 推理节点收图像，发 `/camera/detections`、`/camera/image_annotated`
- 控制节点订阅危险结果

优点：

- 开发最快
- 类型清晰
- 适合本地链路

缺点：

- 跨公网不方便
- 大图高帧率时 DDS 也会吃内存和 CPU

### 4.2 TCP/UDP 裸流传输

原理：

- TCP：可靠、有序、重传
- UDP：低延迟、不保证到达

封装方式：

- 自定义帧头 + 图像字节
- 帧头里包含长度、时间戳、宽高、编码格式

示例头部：

```text
| magic(4B) | ts(8B) | width(2B) | height(2B) | encoding(1B) | payload_len(4B) | payload |
```

适合：

- 边缘设备到自定义服务端
- 对协议完全自主可控的场景

优点：

- 开销小
- 灵活

缺点：

- 要自己处理粘包、分包、重连、乱序、丢包
- 维护成本高

### 4.3 RTSP 视频流

原理：

- 摄像头或 Jetson 输出 H264/H265 视频码流
- 客户端通过 RTSP 拉流
- 常用于“连续预览”

封装方式：

- RTP 负载 H264/H265
- RTSP 负责会话控制

实现思路：

- Jetson 用 GStreamer 输出 RTSP 服务
- 手机端/鸿蒙端/Flutter 用播放器拉流
- 云端也可直接拉流做推理

优点：

- 最适合连续视频预览
- 带宽利用率高
- 生态成熟

缺点：

- 推理前还得解码
- 低延迟调优有门槛
- 结果回传要另开通道

### 4.4 图片帧字节流

原理：

- 每帧单独编码成 JPEG/PNG/WebP
- 通过 HTTP/WebSocket/TCP 逐帧发送

当前仓库实际上已经在用这一类：

- `JetCarEdge`：`sensor_msgs/Image -> JPEG + Base64`
- `JetCarCloud`：接收 JSON 中的 JPEG 字段并解码

优点：

- 实现简单
- 与检测结果 JSON 很容易打包在一起
- 很适合“低帧率上传推理”

缺点：

- Base64 比原始二进制更大
- 高频率上传时带宽和 CPU 都不划算
- 不适合高清连续视频预览

### 4.5 工程建议

把不同链路分工开：

- Jetson 本机：`ROS2 Image` 原始图像
- Jetson 到云端推理结果：`WebSocket JSON`
- Jetson 到移动端预览：`RTSP` 或 `MJPEG/WS JPEG`
- 云端事件上报：`HTTP/WebSocket JSON`

## 5. 最优实时链路完整梳理

推荐主链路：

### 5.1 摄像头采集

- 摄像头原始输出 `NV12 / YUYV / MJPEG`
- 采集节点统一转成 `BGR8`
- 发布 ROS2 话题 `/camera/image_raw`

### 5.2 YOLO 订阅节点接收帧

- 订阅 `/camera/image_raw`
- 使用 `cv_bridge` 或 numpy 转成 OpenCV 图像
- 做 resize / letterbox / normalize

### 5.3 模型推理并绘框

- 本地加载 YOLO 模型
- Jetson 上优先 TensorRT，其次 Ultralytics
- 输出：
  - 检测框
  - 类别
  - 置信度
  - 危险等级
- 同时生成：
  - `/camera/detections`
  - `/camera/image_annotated`

### 5.4 本地分发

- 安全控制节点订阅 `/camera/detections`
- 命中危险目标时发布 `/jetcar/emergency_stop`
- 本地 UI 或调试工具可订阅 `/camera/image_annotated`

### 5.5 移动端分发

推荐不要把原始 ROS2 图像直接给手机，建议：

- 方案 A：Jetson 启一个 `RTSP` 低码率预览流
- 方案 B：网络分发节点把 `/camera/image_annotated` 压缩后通过 WebSocket 发手机

手机端看到的是：

- 视频预览
- 检测结果列表
- 告警状态

### 5.6 云端分发

推荐只上传：

- 检测结果 JSON
- 告警抓拍图
- 低频抽帧

不建议一直上传全分辨率原始流给云端做主推理。

## 6. 每一步数据转换、风险点与优化手段

### 6.1 典型转换链

#### 转换 1：摄像头原生格式 -> OpenCV Mat

- `NV12/YUYV/MJPEG` -> `BGR`
- 工具：
  - GStreamer
  - OpenCV
  - Jetson 硬件转换

风险：

- 颜色空间转换吃 CPU
- 从 GPU 内存拷到 CPU 内存会增延迟

优化：

- CSI 摄像头优先 `nvarguscamerasrc + nvvidconv`
- 尽量少做重复 `videoconvert`

#### 转换 2：OpenCV Mat -> `sensor_msgs/Image`

- 用 `cv_bridge.cv2_to_imgmsg(frame, encoding="bgr8")`

风险：

- 每帧拷贝一次内存

优化：

- 控制图像尺寸
- 不要在多个节点间重复发布超高清图

#### 转换 3：`sensor_msgs/Image` -> YOLO 输入 tensor

- `imgmsg_to_cv2`
- `BGR -> RGB`
- resize / letterbox
- `numpy -> torch/tensorrt tensor`

风险：

- 频繁 resize
- 推理前预处理过多

优化：

- 采集端就限制到 YOLO 实际需要的分辨率，比如 `640x384` 或 `640x480`
- 固定输入尺寸，减少动态 shape 开销

#### 转换 4：标注图像 -> 压缩图像

- `cv2.imencode(".jpg", frame, [quality])`

风险：

- JPEG 编码本身耗 CPU
- 质量过高导致带宽大

优化：

- 预览流用 `quality=60~75`
- 原图只在抓拍事件时上传

#### 转换 5：压缩图像 -> 字节流 / Base64 / WebSocket JSON

- `bytes -> base64 -> json`

风险：

- Base64 体积膨胀约 33%
- JSON 拼装和解析也有开销

优化：

- 高频预览不要长期用 Base64 JSON
- 高频流推荐 RTSP 或 WebSocket binary frame

### 6.2 最容易卡顿、延迟高、丢帧的坑

1. 采集分辨率过高  
   1080p@30 在 Jetson 上做多次颜色转换和 JPEG 编码很容易顶满 CPU。

2. 一个节点里同时做采集、推理、绘图、上传  
   容易互相阻塞。

3. 每个环节都全帧处理  
   原图、绘图图、上传图都用同一分辨率会非常浪费。

4. Base64 当高频视频协议使用  
   适合抓拍，不适合持续 15~30 FPS 预览。

5. ROS2 队列太大  
   队列一大就不是“实时”，而是“排队看旧帧”。

6. 云端主推理依赖公网  
   网络抖动直接导致漏检或控制滞后。

### 6.3 优化建议

- 推理主输入先定在 `640` 宽
- 推理节点订阅 QoS 用 `best_effort + keep_last(1)`
- 采集、推理、上传拆成独立节点
- 预览流与推理流分开
- 结果优先上传 JSON，不是整帧
- Jetson 模型优先 TensorRT
- 对手机只发送低码率预览图和检测框，不要全量原图

## 7. 最简可运行伪代码 / 节点结构

### 7.1 采集节点

```python
class CameraCaptureNode(Node):
    def __init__(self):
        super().__init__("camera_capture")
        self.pub = self.create_publisher(Image, "/camera/image_raw", 10)
        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture(GST_PIPELINE, cv2.CAP_GSTREAMER)
        self.timer = self.create_timer(1.0 / 30.0, self.on_timer)

    def on_timer(self):
        ok, frame = self.cap.read()
        if not ok:
            return
        msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera_link"
        self.pub.publish(msg)
```

### 7.2 推理节点

```python
class YoloInferenceNode(Node):
    def __init__(self):
        super().__init__("yolo_inference")
        self.bridge = CvBridge()
        self.det_pub = self.create_publisher(String, "/camera/detections", 10)
        self.img_pub = self.create_publisher(Image, "/camera/image_annotated", 10)
        self.stop_pub = self.create_publisher(Bool, "/jetcar/emergency_stop", 10)
        self.sub = self.create_subscription(Image, "/camera/image_raw", self.on_image, 1)
        self.model = load_yolo_model()

    def on_image(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        dets = self.model.detect(frame)
        annotated = draw_boxes(frame, dets)

        self.det_pub.publish(String(data=json.dumps(dets, ensure_ascii=False)))
        self.img_pub.publish(self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8"))

        danger = any(d["label"] in {"person", "pit", "obstacle"} and d["confidence"] > 0.6 for d in dets)
        self.stop_pub.publish(Bool(data=danger))
```

### 7.3 网络传输节点

```python
class PreviewUploadNode(Node):
    def __init__(self):
        super().__init__("preview_upload")
        self.bridge = CvBridge()
        self.sub = self.create_subscription(Image, "/camera/image_annotated", self.on_image, 1)
        self.ws = connect_websocket("ws://cloud-host/ws/preview/car_001")

    def on_image(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        frame = resize_keep_ratio(frame, width=640)
        ok, enc = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 70])
        if not ok:
            return
        self.ws.send_binary(enc.tobytes())
```

### 7.4 云端服务节点

```python
@app.websocket("/ws/preview/{car_id}")
async def preview_socket(ws, car_id):
    await ws.accept()
    while True:
        jpeg_bytes = await ws.receive_bytes()
        await broadcast_to_apps(car_id, jpeg_bytes)

@app.websocket("/ws/inference/{car_id}/app")
async def app_result_socket(ws, car_id):
    await ws.accept()
    register_app(ws, car_id)
```

## 8. 对当前仓库的对应关系

### 8.1 已经具备的部分

`JetCarEdge`

- 已有 ROS2 边缘节点订阅 `/camera/image_raw`
- 已有 `sensor_msgs/Image -> JPEG + Base64` 编码逻辑
- 已有 WebSocket 上传云端和结果回灌能力

`JetCarCloud`

- 已有 WebSocket 接收边缘图像
- 已有 JPEG 解码
- 已有 YOLOv5 / Ultralytics 两种 detector 封装
- 已有向 App 广播结果的 WebSocket 路由

`JetCar`

- 已有 TCP 小车控制能力
- 控制页的视频区目前仍是“等待视频流接入”的占位

### 8.2 还缺的关键环节

1. Jetson 侧真正的摄像头采集节点
2. Jetson 本地 YOLO 推理节点
3. 面向移动端的视频预览通道
4. 检测结果标准消息格式
5. 本地推理结果与云端事件上报的分流逻辑

## 9. 完整落地执行步骤

### 第一步：确认摄像头真实能力

在 Jetson 执行：

```bash
v4l2-ctl --list-devices
v4l2-ctl -d /dev/video0 --list-formats-ext
gst-inspect-1.0 nvarguscamerasrc
```

得到：

- 设备类型
- 原生格式
- 支持分辨率
- 支持帧率

如果这一步之前执行 `ros2` 提示 `command not found`，先不要继续测图像话题，先修好 ROS2 环境。

最常见原因有两个：

1. ROS2 已安装，但当前 shell 没有 `source`
2. ROS2 还没安装，或者安装路径不是默认目录

先在 Jetson 上执行：

```bash
printenv | grep -i ROS
ls /opt/ros
find ~ -maxdepth 3 -name setup.bash 2>/dev/null | grep -E "ros|install"
```

如果看到了例如 `/opt/ros/humble/setup.bash`，就执行：

```bash
source /opt/ros/humble/setup.bash
```

如果你还有自己的工作区，例如：

```bash
source ~/yahboomcar_ws/install/setup.bash
```

然后再验证：

```bash
which ros2
ros2 --help
```

如果 `/opt/ros` 都不存在，说明大概率 ROS2 还没装好，需要先安装 ROS2，再继续下面的图像话题调试。

你当前 Jetson 的现场输出如果满足下面三条：

```bash
printenv | grep -i ROS
ls /opt/ros
find ~ -maxdepth 3 -name setup.bash 2>/dev/null | grep -E "ros|install"
```

结果分别是：

- 没有任何 `ROS_*` 环境变量
- `/opt/ros` 不存在
- 用户目录下也没有 ROS 工作区 `install/setup.bash`

那么就可以直接下结论：

**这台 Jetson 当前还没有可用的 ROS2 运行环境。**

此时正确顺序不是继续查 `/camera/image_raw`，而是：

1. 先确认 Ubuntu 版本
2. 安装对应版本的 ROS2
3. 创建并验证最小 ROS2 工作区
4. 再接摄像头采集节点
5. 最后接 YOLO 推理节点

另外如果命令行前缀出现 `(base)`，说明 Conda 环境处于激活状态。后续跑 ROS2 前建议先退出：

```bash
conda deactivate
```

避免 Python 依赖、`cv_bridge`、`rclpy` 和系统 Python 冲突。

### 当前这台 Jetson 的明确建议

如果现场确认结果是：

- `Ubuntu 20.04.5 LTS`
- 架构：`aarch64`

那么当前最稳妥、最省改造成本的选择就是：

**安装 ROS2 Foxy。**

原因很直接：

- `JetCarEdge` 当前就是按 ROS2 Python 节点方式设计的
- Ubuntu 20.04 对应的二进制安装路径里，Foxy 最常见、资料最多
- 对于你今天的目标“先打通摄像头图像流 + YOLO 推理链路”，Foxy 已经足够

建议安装包至少包括：

```bash
ros-foxy-ros-base
python3-colcon-common-extensions
python3-rosdep
python3-vcstool
ros-foxy-cv-bridge
ros-foxy-image-transport
ros-foxy-compressed-image-transport
```

### 如果 ROS2 在 Docker 容器里

有些 Jetson 小车镜像不是把 ROS2 装在宿主机，而是封装在自动驾驶容器里。  
如果你执行类似：

```bash
./run_docker_autodrive.sh
```

进入容器后，`ros2` 命令可用，而宿主机不可用，那么后续所有 ROS2 调试都应该在容器里进行。

这时注意两点：

1. `ros` 和 `ros2` 不是一回事  
   - `ros -version` 是 ROS1 风格命令，ROS2 下本来就可能不存在
   - `ros2 version` 也不是有效子命令

2. 正确验证方式应改成：

```bash
which ros2
printenv | grep -i ROS
ros2 --help
ros2 doctor
ros2 topic list
```

如果容器里还有工作区，通常还需要再 `source` 一次：

```bash
source /opt/ros/foxy/setup.bash
source /workspace/install/setup.bash
```

其中第二条路径要以容器内实际工作区为准。

如果执行到这一步出现下面这种现象：

```bash
ros2 topic list
/parameter_events
/rosout
```

并且：

```bash
ros2 topic info /camera/image_raw
```

返回 `Unknown topic`，那么含义非常明确：

**ROS2 本身已经正常，但当前容器里还没有任何摄像头驱动节点在运行。**

这时要排查的不是图像编码，而是：

1. 相机驱动节点是否启动
2. 实际图像话题名是不是别的名字
3. 自动驾驶脚本是否只启动了基础环境，没有真正 `launch` 摄像头

优先执行：

```bash
source /install/setup.bash
ros2 node list
ros2 topic list | grep -E "camera|image|astra|rgb|depth"
ros2 pkg list | grep -E "astra|camera|yahboom"
ps -ef | grep -E "astra|camera|yahboom" | grep -v grep
```

在你当前这台 Yahboom + AstraPlus 车上，已经实机确认：

```bash
find $(ros2 pkg prefix astra_camera)/share/astra_camera -maxdepth 2 -type f | grep launch
```

返回的实际相机启动文件里，存在的是：

```bash
astro_pro_plus.launch.xml
```

不是很多人直觉里会输入的：

```bash
astra_pro_plus.launch.xml
```

也就是说，这个环境里厂商提供的 launch 文件名本身就是 `astro_pro_plus.launch.xml`。  
如果输成 `astra_pro_plus.launch.xml`，会得到“file not found”，这不是驱动缺失，而只是文件名不一致。

### 第二步：打通 ROS2 图像采集

目标：

- 稳定发布 `/camera/image_raw`
- 能跑到 `640x480@15/30fps`

验证：

```bash
ros2 topic list
ros2 topic hz /camera/image_raw
ros2 topic echo /camera/image_raw/header --once
```

### 第三步：先做 Jetson 本地 YOLO 节点

目标：

- 订阅 `/camera/image_raw`
- 发布 `/camera/detections`
- 发布 `/camera/image_annotated`

验证：

- 本地终端打印检测结果
- 本地看标注图像是否连续

### 第四步：接入本地安全控制

目标：

- 危险目标触发 `/jetcar/emergency_stop`
- 本地控制逻辑优先级高于云端指令

### 第五步：给移动端加预览通道

建议优先级：

1. 先做低帧率 JPEG/WebSocket 预览，开发快
2. 再升级 RTSP/H264 预览，体验更好

### 第六步：给云端加结果与事件上报

上传内容建议：

- 检测结果 JSON
- 危险帧抓拍图
- 低频巡检图

不建议长期上传全分辨率视频做主推理。

### 第七步：最后再决定是否保留云端推理

建议策略：

- 本地 YOLO：主推理
- 云端 YOLO：复核或增强分析
- 手机端：只展示，不做主推理

## 10. 最终推荐架构

一句话总结：

`摄像头采集放 Jetson，主 YOLO 推理放 Jetson，ROS2 负责本地实时分发，移动端看低码率预览，云端只收结果和事件。`

这是当前 JetCar 这个“巡检小车 + ROS2 + Jetson + 移动端控制”场景下，实时性、稳定性、工程复杂度最平衡的方案。
