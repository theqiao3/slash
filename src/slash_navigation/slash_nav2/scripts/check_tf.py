#!/usr/bin/env python3
"""
TF关系检查脚本

用于检查Slash机器人的TF树是否正确配置
"""

import rclpy
from rclpy.node import Node
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import time


class TFChecker(Node):
    def __init__(self):
        super().__init__('tf_checker')
        
        # 创建TF buffer和listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # 需要检查的TF关系
        self.required_frames = {
            'lidar_odom': 'tianracer/odom',           # FAST_LIO提供
            'tianracer/odom': 'tianracer/base_link',     # FAST_LIO提供
            'tianracer/base_link': 'lidar_link'  # 静态TF
        }
        
        # 可选的TF关系
        self.optional_frames = {
            'tianracer/base_link': 'imu_link',
            'tianracer/base_link': 'base_footprint'
        }
        
        self.get_logger().info('TF检查器已启动，等待TF数据...')
        time.sleep(2)  # 等待TF数据
        
    def check_transform(self, parent, child, required=True):
        """检查两个frame之间的变换"""
        try:
            # 尝试查找变换
            transform = self.tf_buffer.lookup_transform(
                parent,
                child,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            
            # 提取变换信息
            trans = transform.transform.translation
            rot = transform.transform.rotation
            
            self.get_logger().info(
                f'✓ {parent} -> {child}:\n'
                f'  平移: [{trans.x:.3f}, {trans.y:.3f}, {trans.z:.3f}]\n'
                f'  旋转: [{rot.x:.3f}, {rot.y:.3f}, {rot.z:.3f}, {rot.w:.3f}]'
            )
            return True
            
        except TransformException as ex:
            if required:
                self.get_logger().error(f'✗ {parent} -> {child}: {ex}')
            else:
                self.get_logger().warn(f'○ {parent} -> {child}: 未找到（可选）')
            return False
    
    def check_all_transforms(self):
        """检查所有必需的变换"""
        self.get_logger().info('\n========== 检查必需的TF关系 ==========')
        
        all_ok = True
        for parent, child in self.required_frames.items():
            if not self.check_transform(parent, child, required=True):
                all_ok = False
        
        self.get_logger().info('\n========== 检查可选的TF关系 ==========')
        for parent, child in self.optional_frames.items():
            self.check_transform(parent, child, required=False)
        
        self.get_logger().info('\n========== 检查结果 ==========')
        if all_ok:
            self.get_logger().info('✓ 所有必需的TF关系都正常！')
        else:
            self.get_logger().error('✗ 存在缺失的TF关系，请检查！')
            self.print_troubleshooting_tips()
        
        return all_ok
    
    def print_troubleshooting_tips(self):
        """打印故障排除提示"""
        self.get_logger().info('\n========== 故障排除提示 ==========')
        self.get_logger().info(
            '1. 检查FAST_LIO是否在运行:\n'
            '   ros2 node list | grep fast_lio\n'
            '\n'
            '2. 检查AMCL是否在运行:\n'
            '   ros2 node list | grep amcl\n'
            '\n'
            '3. 查看当前所有frame:\n'
            '   ros2 run tf2_ros tf2_echo <parent_frame> <child_frame>\n'
            '\n'
            '4. 生成TF树图:\n'
            '   ros2 run tf2_tools view_frames\n'
            '   evince frames.pdf\n'
            '\n'
            '5. 检查静态TF发布器:\n'
            '   ros2 topic echo /tf_static\n'
            '\n'
            '6. 可能需要添加静态TF:\n'
            '   ros2 run tf2_ros static_transform_publisher \\\n'
            '       0 0 0.2 0 0 0 base_link lidar_link\n'
        )


def main(args=None):
    rclpy.init(args=args)
    
    checker = TFChecker()
    
    try:
        # 执行检查
        checker.check_all_transforms()
        
        # 持续监控（可选）
        checker.get_logger().info('\n持续监控TF（按Ctrl+C退出）...')
        rclpy.spin(checker)
        
    except KeyboardInterrupt:
        pass
    finally:
        checker.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
