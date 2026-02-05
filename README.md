# yolo26_ros2

A ROS 2 package for real-time object detection using YOLO26 models with USB camera integration. This package contains a single node that subscribes to raw Image messages, detects objects, and publishes DetectionArray messages and annotated images.

The annotated images are published in both raw and compressed forms for easier consumption across networks.

## Prerequisites

This package requires ROS 2 Humble or later. Follow the official installation guide:

- [ROS 2 Humble Installation](https://docs.ros.org/en/humble/Installation.html)

After installing ROS 2, source the setup file:

```bash
source /opt/ros/humble/setup.bash
```

All other dependencies will be installed during the compilation process.

## Installation

1. Create a ROS 2 workspace (if you don't have one):

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

2. Clone this repository:

```bash
git clone https://github.com/mikelikesrobots/yolo26_ros2.git
```

3. Install dependencies:

```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

4. Build the package:

```bash
colcon build --symlink-install
```

5. Source the workspace:

```bash
source ~/ros2_ws/install/setup.bash
```

## YOLO Model Setup

There are two options for downloading YOLO models. First, the `ultralytics` package will automatically download and run models that aren't on the local file system. This is an extremely convenient way to deploy, but due to the structure of ROS 2 workspaces, a clean build will delete any downloaded models.

If you want to download the models for persistent use, navigate to TODO and download the required model, then place it in the `models/` directory of this package. You can then rebuild the package to copy it to the correct location. Make sure that the model has the same name as specified for the launch parameters, such as `yolo26n.pt`.

In addition, you can load your own model by following this same pattern. This process is out of scope for this package.

## Usage

### Running with Launch File (Recommended)

Launch both the USB camera and YOLO detection node:

```bash
ros2 launch yolo26_ros2 yolo_camera.launch.py
```

### Running Nodes Individually

#### 1. Start the USB Camera

```bash
ros2 run usb_cam usb_cam_node_exe
```

#### 2. Start the YOLO Detection Node

```bash
ros2 run yolo26_ros2 yolo26_ros2
```

## Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/detections` | `vision_msgs/msg/Detection2DArray` | Array of 2D object detections |
| `/image_annotated` | `sensor_msgs/msg/Image` | Uncompressed annotated image with bounding boxes |
| `/image_annotated/compressed` | `sensor_msgs/msg/CompressedImage` | JPEG compressed annotated image |

## Subscribed Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/image_raw` | `sensor_msgs/msg/Image` | Input camera images for detection |

## Visualization

### Using RViz2

```bash
rviz2
```

Add an Image display and set the topic to `/image_annotated`.

### Using rqt_image_view

```bash
ros2 run rqt_image_view rqt_image_view image_annotated
```

### Viewing Detection Messages

```bash
ros2 topic echo /detections
```

## License

MIT-0
