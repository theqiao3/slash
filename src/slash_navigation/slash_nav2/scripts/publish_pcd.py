#!/usr/bin/env python3
"""
PCD点云发布节点 - 将PCD文件发布为ROS 2 PointCloud2 消息

用法:
    ros2 run slash_nav2 publish_pcd.py --pcd-file <path/to/file.pcd>
    
参数:
    --pcd-file: PCD文件路径（必需）
    --topic: 发布话题名称，默认 /pcd_map
    --frame-id: 坐标系，默认 map
    --rate: 发布频率（Hz），默认 1
    --publish-once: 只发布一次后退出
    --roll: 绕X轴旋转角度（度），用于纠正点云倾斜，默认 0
    --pitch: 绕Y轴旋转角度（度），默认 0
    --yaw: 绕Z轴旋转角度（度），默认 0

功能说明:
    - 支持从PCD文件加载点云数据
    - 支持坐标系旋转纠正（用于纠正传感器安装倾斜导致的偏转）
    - 将点云发布为ROS 2 PointCloud2消息

RViz2配置:
    1. 在RViz2中添加 PointCloud2 显示
    2. 设置话题为 /pcd_map
    3. 根据需要调整颜色和点大小

坐标系旋转纠正示例:
    # 如果点云绕X轴发生了5度的偏转，使用 --roll -5 来纠正
    ros2 run slash_nav2 publish_pcd.py --pcd-file map.pcd --roll -5 --publish-once
    
    # 多轴旋转纠正
    ros2 run slash_nav2 publish_pcd.py --pcd-file map.pcd --roll -5 --pitch 2 --yaw 1
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from rclpy.qos import QoSProfile, DurabilityPolicy
import argparse
import numpy as np
from pathlib import Path
from scipy.spatial.transform import Rotation

try:
    import open3d as o3d
    HAS_OPEN3D = True
except ImportError:
    HAS_OPEN3D = False

try:
    import pcl
    HAS_PCL = True
except ImportError:
    HAS_PCL = False


class PCDPublisher(Node):
    """PCD文件发布节点"""
    
    def __init__(self, pcd_file, topic='/pcd_map', frame_id='map', 
                 rate=1.0, publish_once=False, roll=0.0, pitch=0.0, yaw=0.0):
        super().__init__('pcd_publisher')
        
        self.pcd_file = Path(pcd_file)
        self.frame_id = frame_id
        self.publish_once = publish_once
        self.published = False
        
        # 旋转角度（弧度）
        self.roll = np.radians(roll)
        self.pitch = np.radians(pitch)
        self.yaw = np.radians(yaw)
        
        # 发布器: 使用 transient_local durability 以支持 late subscribers（RViz 等）
        qos = QoSProfile(depth=1)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.publisher = self.create_publisher(PointCloud2, topic, qos_profile=qos)
        
        # 定时器（发布频率）
        self.timer = self.create_timer(1.0 / rate, self.timer_callback)
        
        # 加载PCD文件
        self.get_logger().info(f"加载PCD文件: {self.pcd_file}")
        self.points = self.load_pcd()
        
        if self.points is None or len(self.points) == 0:
            self.get_logger().error(f"无法加载PCD文件或文件为空: {self.pcd_file}")
            raise RuntimeError("Failed to load PCD file")
        
        # 应用坐标系旋转纠正
        if roll != 0.0 or pitch != 0.0 or yaw != 0.0:
            self.points = self.apply_rotation(self.points)
            self.get_logger().info(f"已应用旋转纠正: Roll={roll}°, Pitch={pitch}°, Yaw={yaw}°")
        
        self.get_logger().info(f"✓ 成功加载PCD文件")
        self.get_logger().info(f"  点云数量: {len(self.points)}")
        self.get_logger().info(f"  坐标范围:")
        self.get_logger().info(f"    X: [{self.points[:, 0].min():.2f}, {self.points[:, 0].max():.2f}]")
        self.get_logger().info(f"    Y: [{self.points[:, 1].min():.2f}, {self.points[:, 1].max():.2f}]")
        self.get_logger().info(f"    Z: [{self.points[:, 2].min():.2f}, {self.points[:, 2].max():.2f}]")
        self.get_logger().info(f"  发布话题: {topic}")
        self.get_logger().info(f"  坐标系: {frame_id}")
    
    def apply_rotation(self, points):
        """
        对点云应用坐标系旋转纠正
        
        参数:
            points: Nx3 的numpy数组，包含 [x, y, z] 坐标
        
        返回:
            旋转后的点云数组
        """
        # 使用欧拉角 (roll, pitch, yaw) 创建旋转
        # 旋转顺序：先绕Z轴(yaw)，再绕Y轴(pitch)，最后绕X轴(roll)
        rotation = Rotation.from_euler('xyz', [self.roll, self.pitch, self.yaw], degrees=False)
        rotated_points = rotation.apply(points)
        return rotated_points
    
    def load_pcd(self):
        """加载PCD文件"""
        if not self.pcd_file.exists():
            self.get_logger().error(f"文件不存在: {self.pcd_file}")
            return None
        
        try:
            if HAS_OPEN3D:
                self.get_logger().info("使用 open3d 加载PCD文件")
                pcd = o3d.io.read_point_cloud(str(self.pcd_file))
                return np.asarray(pcd.points, dtype=np.float32)
            elif HAS_PCL:
                self.get_logger().info("使用 python-pcl 加载PCD文件")
                cloud = pcl.load(str(self.pcd_file))
                return np.asarray(cloud, dtype=np.float32)
            else:
                self.get_logger().warning("未安装 open3d 或 python-pcl，尝试手动解析")
                return self.load_pcd_manual()
        except Exception as e:
            self.get_logger().error(f"加载失败: {e}")
            return None
    
    def load_pcd_manual(self):
        """手动解析PCD文件（备用方案）"""
        points = []
        try:
            with open(self.pcd_file, 'r') as f:
                skip_header = False
                for line in f:
                    line = line.strip()
                    if line.startswith('END'):
                        break
                    if line.startswith('DATA'):
                        skip_header = True
                        continue
                    if skip_header and line and not line.startswith('VERSION'):
                        try:
                            x, y, z = map(float, line.split()[:3])
                            points.append([x, y, z])
                        except ValueError:
                            continue
            
            if points:
                return np.array(points, dtype=np.float32)
        except Exception as e:
            self.get_logger().error(f"手动解析失败: {e}")
        
        return None
    
    def timer_callback(self):
        """定时回调函数"""
        if self.publish_once and self.published:
            return
        
        # 创建PointCloud2消息
        msg = self.create_point_cloud2_msg(self.points)
        self.publisher.publish(msg)
        
        if not self.published:
            self.get_logger().info(f"已发布点云到话题: /pcd_map")
            self.published = True
            
            if self.publish_once:
                self.get_logger().info("发布完成，节点将退出")
                rclpy.shutdown()
    
    def create_point_cloud2_msg(self, points):
        """
        将点云数组转换为PointCloud2消息
        
        参数:
            points: Nx3 的numpy数组，包含 [x, y, z] 坐标
        
        返回:
            PointCloud2 消息对象
        """
        # 确保点云是float32类型
        points = np.asarray(points, dtype=np.float32)
        
        # 创建消息头
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.frame_id
        
        # 定义字段
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        
        # 创建点云数据（12字节/点：3个float32）
        point_cloud_data = np.zeros(len(points), dtype=np.dtype([
            ('x', np.float32),
            ('y', np.float32),
            ('z', np.float32),
        ]))
        
        point_cloud_data['x'] = points[:, 0]
        point_cloud_data['y'] = points[:, 1]
        point_cloud_data['z'] = points[:, 2]
        
        # 创建PointCloud2消息
        msg = PointCloud2(
            header=header,
            height=1,
            width=len(points),
            is_dense=True,
            is_bigendian=False,
            fields=fields,
            point_step=12,  # 3个float32 = 12字节
            row_step=12 * len(points),
            data=point_cloud_data.tobytes()
        )
        
        return msg


def main():
    parser = argparse.ArgumentParser(
        description='将PCD点云文件发布为ROS 2 PointCloud2消息',
        epilog="""
示例:
  ros2 run slash_nav2 publish_pcd.py --pcd-file /path/to/map.pcd
  ros2 run slash_nav2 publish_pcd.py --pcd-file map.pcd --topic /cloud_map --rate 10
  ros2 run slash_nav2 publish_pcd.py --pcd-file map.pcd --publish-once
  ros2 run slash_nav2 publish_pcd.py --pcd-file map.pcd --roll -5 (纠正绕X轴-5度的偏转)
  ros2 run slash_nav2 publish_pcd.py --pcd-file map.pcd --roll -5 --pitch 2 --yaw 1
        """
    )
    parser.add_argument('--pcd-file', required=True, help='PCD文件路径（必需）')
    parser.add_argument('--topic', default='/pcd_map', help='发布话题，默认 /pcd_map')
    parser.add_argument('--frame-id', default='map', help='坐标系，默认 map')
    parser.add_argument('--rate', type=float, default=1.0, help='发布频率（Hz），默认1')
    parser.add_argument('--publish-once', action='store_true', help='只发布一次后退出')
    parser.add_argument('--roll', type=float, default=0.0, 
                       help='绕X轴旋转角度（度），默认0。用于纠正点云倾斜')
    parser.add_argument('--pitch', type=float, default=0.0, 
                       help='绕Y轴旋转角度（度），默认0')
    parser.add_argument('--yaw', type=float, default=0.0, 
                       help='绕Z轴旋转角度（度），默认0')
    
    # 允许忽略 ros2 launch 注入的 ROS-specific args（例如 --ros-args）
    args, _ = parser.parse_known_args()
    
    rclpy.init()
    
    try:
        node = PCDPublisher(
            pcd_file=args.pcd_file,
            topic=args.topic,
            frame_id=args.frame_id,
            rate=args.rate,
            publish_once=args.publish_once,
            roll=args.roll,
            pitch=args.pitch,
            yaw=args.yaw
        )
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n已停止节点")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
