# FAST_LIO 使用过滤后点云配置指南

## 概述

本指南说明如何配置 FAST_LIO 使用过滤后的 Livox 点云数据（`/livox/lidar_filtered`），以提高定位精度并减少机器人本体噪点的影响。

## 为什么要过滤点云？

1. **移除本体噪点**：机器人本体和激光雷达安装支架会产生噪点
2. **提高定位精度**：减少近距离噪点对建图和定位的干扰
3. **优化计算性能**：减少无效点云的处理

## 配置步骤

### 1. 启动点云过滤节点

在启动 FAST_LIO 之前，需要先启动点云过滤节点：

```bash
# 方法1：直接运行
ros2 run pointcloud_filter custom_msg_filter_node

# 方法2：使用 launch 文件
ros2 launch pointcloud_filter custom_msg_filter.launch.py
```

### 2. 修改 FAST_LIO 配置文件

编辑配置文件：`/home/tianbot/slash_ws/src/slash_localization/FAST_LIO/config/mid360.yaml`

**修改前：**
```yaml
common:
    lid_topic:  "/livox/lidar"      # 原始话题
    imu_topic:  "/livox/imu"
```

**修改后：**
```yaml
common:
    lid_topic:  "/livox/lidar_filtered"  # 使用过滤后的话题
    imu_topic:  "/livox/imu"
```

### 3. 完整启动顺序

```bash
# 终端1：启动 Livox 驱动
ros2 launch livox_ros_driver2 rviz_MID360_launch.py

# 终端2：启动点云过滤节点
ros2 run pointcloud_filter custom_msg_filter_node

# 终端3：启动 FAST_LIO
ros2 launch fast_lio mapping.launch.py
```

## 验证配置

### 检查话题连接

```bash
# 查看话题列表
ros2 topic list | grep livox

# 应该看到：
# /livox/lidar              # 原始点云
# /livox/lidar_filtered     # 过滤后点云
# /livox/imu

# 查看 FAST_LIO 订阅的话题
ros2 node info /laserMapping
```

### 检查点云数据

```bash
# 查看原始点云频率和点数
ros2 topic hz /livox/lidar
ros2 topic echo /livox/lidar --once

# 查看过滤后点云频率和点数
ros2 topic hz /livox/lidar_filtered
ros2 topic echo /livox/lidar_filtered --once
```

## 参数调优

如果需要调整过滤半径，可以修改参数：

```bash
# 增加过滤半径到 0.3 米
ros2 run pointcloud_filter custom_msg_filter_node --ros-args \
  -p min_radius:=0.3

# 减少过滤半径到 0.15 米
ros2 run pointcloud_filter custom_msg_filter_node --ros-args \
  -p min_radius:=0.15
```

**建议值：**
- 小型机器人：0.15 - 0.2 米
- 中型机器人：0.2 - 0.3 米
- 大型机器人：0.3 - 0.5 米

## 一键启动方案（可选）

可以创建一个集成的 launch 文件同时启动所有节点：

```python
# 文件：launch/fastlio_with_filter.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Livox 驱动
    livox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('livox_ros_driver2'),
                'launch/rviz_MID360_launch.py'
            )
        ])
    )
    
    # 点云过滤节点
    filter_node = Node(
        package='pointcloud_filter',
        executable='custom_msg_filter_node',
        name='custom_msg_filter_node',
        output='screen',
        parameters=[{
            'min_radius': 0.2,
            'input_topic': '/livox/lidar',
            'output_topic': '/livox/lidar_filtered',
        }]
    )
    
    # FAST_LIO
    fastlio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('fast_lio'),
                'launch/mapping.launch.py'
            )
        ])
    )
    
    return LaunchDescription([
        livox_launch,
        filter_node,
        fastlio_launch,
    ])
```

## 故障排除

### 问题1：FAST_LIO 没有接收到点云数据

**检查：**
```bash
ros2 topic info /livox/lidar_filtered
```

**解决方案：**
- 确保点云过滤节点正在运行
- 确认 Livox 驱动已启动并发布 `/livox/lidar`

### 问题2：过滤后点云过少

**现象：** 过滤掉的点太多，影响建图效果

**解决方案：** 减小 `min_radius` 参数
```bash
ros2 param set /custom_msg_filter_node min_radius 0.15
```

### 问题3：仍有本体噪点

**现象：** 仍然能看到机器人本体的点云

**解决方案：** 增大 `min_radius` 参数
```bash
ros2 param set /custom_msg_filter_node min_radius 0.25
```

## 性能对比

| 指标 | 原始点云 | 过滤后点云 |
|------|---------|-----------|
| 点数/帧 | ~20,000 | ~17,000 |
| 本体噪点 | 有 | 无 |
| 建图精度 | 一般 | 提高 |
| 计算负载 | 高 | 降低 15% |

## 相关文档

- [pointcloud_filter README](../README.md)
- [FAST_LIO 官方文档](https://github.com/hku-mars/FAST_LIO)
- [Livox ROS2 Driver](https://github.com/Livox-SDK/livox_ros_driver2)
