# Camera Calibration and Configuration Guide

This guide explains how to extract camera intrinsics from ROS 2 topics and generate ORB-SLAM3 configuration files.

Chinese version: [CAMERA_CALIBRATION_zh.md](CAMERA_CALIBRATION_zh.md)

## Quick Start

### 1. Launch the camera node

Ensure your camera node is running and publishing topics:

```bash
ros2 topic list | grep camera_info
```

You should see:
```
/camera/color/camera_info
/camera/depth/camera_info
```

### 2. Generate a configuration file

```bash
cd ~/ros2_ws/src/orbslam3_ros
python3 scripts/generate_camera_config.py --output config/my_camera.yaml
```

### 3. Verify the depth stream

```bash
python3 scripts/verify_camera_config.py
```

### 4. Launch ORB-SLAM3

```bash
ros2 launch orbslam3_ros orbslam3_realtime.launch.py \
  config_file:=config/my_camera.yaml
```

## Script Reference

### generate_camera_config.py

Generates an ORB-SLAM3 configuration file from ROS 2 `camera_info` topics.

The script waits for both color and depth `CameraInfo` messages before it writes the file, but it uses the color camera intrinsics and distortion parameters to build the output.

**Parameters:**

- `--color-topic`: Color camera info topic (default: `/camera/color/camera_info`)
- `--depth-topic`: Depth camera info topic (default: `/camera/depth/camera_info`)
- `--output` / `-o`: Output config file path (default: `camera_config.yaml`)
- `--camera-name`: Camera parameter prefix (default: `Camera1`)
- `--timeout`: Timeout in seconds (default: 10.0)

**Examples:**

```bash
# Basic usage
python3 scripts/generate_camera_config.py -o config/realsense.yaml

# Custom topics
python3 scripts/generate_camera_config.py \
  --color-topic /my_camera/rgb/camera_info \
  --depth-topic /my_camera/depth/camera_info \
  --output config/custom_camera.yaml

# Extended timeout
python3 scripts/generate_camera_config.py -o config/camera.yaml --timeout 30
```

### verify_camera_config.py

Analyzes depth image data and suggests an `RGBD.DepthMapFactor` value.

The recommendation is a heuristic based on the observed depth range. Confirm the result against the camera driver documentation when available.

The script outputs:
- Depth image encoding format
- Statistics of valid depth values (min, max, mean, median)
- Recommended `RGBD.DepthMapFactor` value

## Camera Intrinsics

### camera_info Message Structure

```yaml
k: [fx,  0, cx,
     0, fy, cy,
     0,  0,  1]

d: [k1, k2, p1, p2, k3]
```

**K Matrix (Intrinsic Matrix):**
- `k[0]` = fx (focal length in x)
- `k[4]` = fy (focal length in y)
- `k[2]` = cx (principal point x)
- `k[5]` = cy (principal point y)

**D Array (Distortion Coefficients):**
- `d[0]` = k1 (radial distortion coefficient 1)
- `d[1]` = k2 (radial distortion coefficient 2)
- `d[2]` = p1 (tangential distortion coefficient 1)
- `d[3]` = p2 (tangential distortion coefficient 2)
- `d[4]` = k3 (radial distortion coefficient 3)

### Manual Intrinsics Retrieval

To manually inspect camera intrinsics:

```bash
# Get color camera intrinsics
ros2 topic echo /camera/color/camera_info --once

# Get depth camera intrinsics
ros2 topic echo /camera/depth/camera_info --once
```

## Configuration Parameters

### Required Adjustments

#### RGBD.DepthMapFactor

Depth scaling factor, depends on depth camera output unit:

- **RealSense D435/D455**: `1000.0` (depth in millimeters)
- **Astra Pro**: `1000.0` (depth in millimeters)
- **Kinect v1**: `5000.0`
- **Kinect v2**: `1000.0`
- **ROS bag recordings**: typically `1.0` (already converted to meters)

**Determination method:**

Run the `verify_camera_config.py` script for automatic analysis and recommendations.

Or check manually:
```bash
ros2 topic echo /camera/depth/image_raw --once
```

If depth values are in the 500-5000 range, use `1000.0`;
If depth values are in the 0.5-5.0 range, use `1.0`.

#### Camera.fps

Set this to the actual camera frame rate, such as 15, 30, or 60.

### Optional Adjustments

#### ORB Feature Extraction Parameters

Adjust based on scene characteristics:

**High-texture scenes** (indoor environments with rich details):
```yaml
ORBextractor.nFeatures: 1000
ORBextractor.iniThFAST: 20
ORBextractor.minThFAST: 7
```

**Low-texture scenes** (corridors, white walls):
```yaml
ORBextractor.nFeatures: 1500-2000
ORBextractor.iniThFAST: 10-15
ORBextractor.minThFAST: 5
```

#### Stereo.ThDepth

Depth validity threshold in baseline multiples. The default value `40.0` suits most scenarios.

#### Stereo.b

Stereo camera baseline. For RGB-D cameras, the generated configuration uses a nominal value of `0.075` m.

## Troubleshooting

### Topic Not Published

**Error:**
```
WARNING: topic [/camera/color/camera_info] does not appear to be published yet
```

**Solutions:**
1. Ensure camera node is running
2. Check topic names: `ros2 topic list`
3. Wait for camera initialization (usually takes a few seconds)

### All Distortion Coefficients Are Zero

Some camera drivers output rectified images with zero distortion coefficients, which is normal.

Verify:
- Keep distortion parameters at 0
- Set `Camera.type` to `"PinHole"`

### SLAM Tracking Lost

**Possible Causes:**

1. **Incorrect depth scale factor**
   - Run `verify_camera_config.py` for verification
   - Check `RGBD.DepthMapFactor` setting

2. **Too few features**
   - Increase `ORBextractor.nFeatures` to 1500-2000
   - Lower `ORBextractor.iniThFAST` to 10-15

3. **Inaccurate camera intrinsics**
   - Re-run `generate_camera_config.py`
   - Verify topic names are correct

4. **Image synchronization issues**
   - Check if timestamps are synchronized
   - Verify TF tree is correct

### Too Many Invalid Depth Pixels

**Causes:**
- Objects outside camera's effective range
- Transparent or reflective surfaces
- Poor lighting conditions

**Solutions:**
- Ensure scene is within camera working distance (Astra Pro: 0.6m-8m)
- Avoid pointing directly at glass, mirrors, or reflective objects
- Improve ambient lighting

## Configuration Examples

### Astra Pro Configuration

```yaml
Camera1.fx: 570.342205
Camera1.fy: 570.342205
Camera1.cx: 319.500000
Camera1.cy: 239.500000

Camera1.k1: 0.000000
Camera1.k2: 0.000000
Camera1.p1: 0.000000
Camera1.p2: 0.000000
Camera1.k3: 0.000000

Camera.width: 640
Camera.height: 480
Camera.fps: 30

RGBD.DepthMapFactor: 1000.0
```

### RealSense D435 Configuration

```yaml
Camera1.fx: 615.123
Camera1.fy: 615.789
Camera1.cx: 320.456
Camera1.cy: 240.234

Camera1.k1: 0.0
Camera1.k2: 0.0
Camera1.p1: 0.0
Camera1.p2: 0.0
Camera1.k3: 0.0

Camera.width: 640
Camera.height: 480
Camera.fps: 30

RGBD.DepthMapFactor: 1000.0
```

## Resources

- [ORB-SLAM3 Official Repository](https://github.com/UZ-SLAMLab/ORB_SLAM3)
- [ROS2 sensor_msgs/CameraInfo](https://docs.ros.org/en/rolling/p/sensor_msgs/interfaces/msg/CameraInfo.html)
- [OpenCV Camera Calibration Tutorial](https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html)
