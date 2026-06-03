# 相机内参获取与配置指南

本指南介绍如何从 ROS 2 相机话题获取内参并生成 ORB-SLAM3 配置文件。

English version: [CAMERA_CALIBRATION.md](CAMERA_CALIBRATION.md)

## 快速开始

### 1. 启动相机节点

确保相机节点正在运行并发布话题：

```bash
ros2 topic list | grep camera_info
```

应该看到：
```
/camera/color/camera_info
/camera/depth/camera_info
```

### 2. 生成配置文件

```bash
cd ~/ros2_ws/src/orbslam3_ros
python3 scripts/generate_camera_config.py --output config/my_camera.yaml
```

### 3. 验证深度流

```bash
python3 scripts/verify_camera_config.py
```

### 4. 启动 ORB-SLAM3

```bash
ros2 launch orbslam3_ros orbslam3_realtime.launch.py \
  config_file:=config/my_camera.yaml
```

## 脚本说明

### generate_camera_config.py

从 ROS 2 `camera_info` 话题生成 ORB-SLAM3 配置文件。

该脚本会等待彩色和深度 `CameraInfo` 消息都到达后再写出配置文件，但实际写入参数时使用的是彩色相机的内参和畸变参数。

**参数：**

- `--color-topic`：彩色相机信息话题（默认：`/camera/color/camera_info`）
- `--depth-topic`：深度相机信息话题（默认：`/camera/depth/camera_info`）
- `--output` / `-o`：输出配置文件路径（默认：`camera_config.yaml`）
- `--camera-name`：相机参数前缀（默认：`Camera1`）
- `--timeout`：超时时间秒数（默认：10.0）

**示例：**

```bash
# 基本用法
python3 scripts/generate_camera_config.py -o config/realsense.yaml

# 自定义话题
python3 scripts/generate_camera_config.py \
  --color-topic /my_camera/rgb/camera_info \
  --depth-topic /my_camera/depth/camera_info \
  --output config/custom_camera.yaml

# 增加超时时间
python3 scripts/generate_camera_config.py -o config/camera.yaml --timeout 30
```

### verify_camera_config.py

分析深度图像数据，并给出 `RGBD.DepthMapFactor` 的建议值。

该建议基于观测到的深度范围，是经验性判断。若驱动文档可用，应优先与文档核对。

脚本会输出：
- 深度图像编码格式
- 有效深度值的统计信息（最小值、最大值、平均值、中位数）
- 推荐的 `RGBD.DepthMapFactor` 值

## 相机内参说明

### camera_info 消息结构

```yaml
k: [fx,  0, cx,
     0, fy, cy,
     0,  0,  1]

d: [k1, k2, p1, p2, k3]
```

**K 矩阵（内参矩阵）：**
- `k[0]` = fx（x 方向焦距）
- `k[4]` = fy（y 方向焦距）
- `k[2]` = cx（主点 x 坐标）
- `k[5]` = cy（主点 y 坐标）

**D 数组（畸变系数）：**
- `d[0]` = k1（径向畸变系数 1）
- `d[1]` = k2（径向畸变系数 2）
- `d[2]` = p1（切向畸变系数 1）
- `d[3]` = p2（切向畸变系数 2）
- `d[4]` = k3（径向畸变系数 3）

### 手动获取内参

如果需要手动查看相机内参：

```bash
# 获取彩色相机内参
ros2 topic echo /camera/color/camera_info --once

# 获取深度相机内参
ros2 topic echo /camera/depth/camera_info --once
```

## 配置文件参数

### 必须调整的参数

#### RGBD.DepthMapFactor

深度缩放因子，取决于深度相机输出单位：

- **RealSense D435/D455**: `1000.0`（深度单位为毫米）
- **Astra Pro**: `1000.0`（深度单位为毫米）
- **Kinect v1**: `5000.0`
- **Kinect v2**: `1000.0`
- **ROS bag 录制数据**: 通常是 `1.0`（已转换为米）

**确定方法：**

运行 `verify_camera_config.py` 脚本会自动分析并给出建议。

或手动检查：
```bash
ros2 topic echo /camera/depth/image_raw --once
```

如果深度值在 500-5000 范围，使用 `1000.0`；
如果深度值在 0.5-5.0 范围，使用 `1.0`。

#### Camera.fps

设置为相机实际帧率，例如 15、30 或 60。

### 可选调整的参数

#### ORB 特征提取参数

根据场景调整：

**高纹理场景**（室内、有丰富细节）：
```yaml
ORBextractor.nFeatures: 1000
ORBextractor.iniThFAST: 20
ORBextractor.minThFAST: 7
```

**低纹理场景**（走廊、白墙）：
```yaml
ORBextractor.nFeatures: 1500-2000
ORBextractor.iniThFAST: 10-15
ORBextractor.minThFAST: 5
```

#### Stereo.ThDepth

深度有效范围阈值，单位为基线倍数。默认值 `40.0` 适用于大多数场景。

#### Stereo.b

立体相机基线。对于 RGB-D 相机，生成的配置文件使用标称值 `0.075` m。

## 常见问题

### 话题未发布

**错误：**
```
WARNING: topic [/camera/color/camera_info] does not appear to be published yet
```

**解决方法：**
1. 确保相机节点已启动
2. 检查话题名称：`ros2 topic list`
3. 等待相机初始化（通常需要几秒）

### 畸变参数全为 0

某些相机驱动输出的是已校正图像，畸变系数为 0 是正常的。

确认：
- 保持畸变参数为 0
- `Camera.type` 设置为 `"PinHole"`

### SLAM 跟踪丢失

**可能原因：**

1. **深度缩放因子错误**
   - 运行 `verify_camera_config.py` 验证
   - 检查 `RGBD.DepthMapFactor` 设置

2. **特征点太少**
   - 增加 `ORBextractor.nFeatures` 到 1500-2000
   - 降低 `ORBextractor.iniThFAST` 到 10-15

3. **相机内参不准确**
   - 重新运行 `generate_camera_config.py`
   - 确认话题名称正确

4. **图像同步问题**
   - 检查时间戳是否同步
   - 验证 TF 树是否正确

### 深度图无效像素过多

**原因：**
- 物体距离超出相机有效范围
- 透明或反光表面
- 光照条件不佳

**解决方法：**
- 确保场景在相机工作距离内（Astra Pro: 0.6m-8m）
- 避免正对玻璃、镜面等反光物体
- 改善环境光照

## 参考示例

### Astra Pro 配置示例

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

### RealSense D435 配置示例

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

## 相关资源

- [ORB-SLAM3 官方仓库](https://github.com/UZ-SLAMLab/ORB_SLAM3)
- [ROS2 sensor_msgs/CameraInfo](https://docs.ros.org/en/rolling/p/sensor_msgs/interfaces/msg/CameraInfo.html)
- [OpenCV 相机标定教程](https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html)
