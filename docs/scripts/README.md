# Scripts / 脚本说明

This directory provides utility scripts for camera configuration generation and depth verification.

本目录提供相机配置生成与深度验证的实用脚本。

## Files / 文件

### Python Scripts / Python 脚本

- **generate_camera_config.py** - Generate an ORB-SLAM3 RGB-D configuration from ROS 2 `camera_info` topics
- **verify_camera_config.py** - Inspect a depth image stream and suggest `RGBD.DepthMapFactor`

### Documentation / 文档

- **[CAMERA_CALIBRATION.md](CAMERA_CALIBRATION.md)** - Camera calibration and configuration guide
- **[CAMERA_CALIBRATION_zh.md](CAMERA_CALIBRATION_zh.md)** - 相机标定与配置指南

## Quick Start / 快速开始

### Generate a configuration file / 生成配置文件

```bash
python3 scripts/generate_camera_config.py --output config/my_camera.yaml
```

### Verify depth scaling / 验证深度缩放

```bash
python3 scripts/verify_camera_config.py
```

## Notes / 说明

- `generate_camera_config.py` waits for both color and depth `camera_info` messages, but it uses the color camera intrinsics to write the ORB-SLAM3 configuration.
- `verify_camera_config.py` subscribes to `/camera/depth/image_raw` and `/camera/color/image_raw` by default.
- The calibration guides describe the expected topics, parameters, and output format in more detail.
