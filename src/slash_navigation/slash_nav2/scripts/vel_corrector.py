#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math

class VelocityCorrectorNode(Node):
    def __init__(self):
        super().__init__('velocity_corrector_node')

        # ================= 参数配置区域 =================
        # 1. 速度限制参数
        self.declare_parameter('min_hardware_speed', 0.8) # 硬件强制的最低速度
        
        # 2. 修正强度系数 (Sensitivity)
        # 该值越大，当速度被强制提升时，对转向角的削弱越狠
        # 建议范围 1.0 ~ 2.0。 1.0 为线性修正，2.0 为平方级修正
        self.declare_parameter('correction_gain', 1.2) 
        
        # 获取参数
        self.min_speed = self.get_parameter('min_hardware_speed').value
        self.gain = self.get_parameter('correction_gain').value

        # ================= 通信配置 =================
        # 订阅 Nav2 原始指令
        self.sub_cmd_vel = self.create_subscription(
            Twist, 
            'cmd_vel', 
            self.cmd_vel_callback, 
            10)

        # 发布修正后的 Twist
        self.pub_cmd_vel_corrective = self.create_publisher(
            Twist, 
            'cmd_vel_corrective', 
            10)

        self.get_logger().info(
            f'速度修正节点已启动: 强制最低速={self.min_speed}m/s, 修正增益={self.gain}\n'
            f'订阅话题: cmd_vel | 发布话题: cmd_vel_corrective'
        )

    def cmd_vel_callback(self, msg):
        # 1. 提取原始数据
        v_in = msg.linear.x
        omega_in = msg.angular.z
        
        # 初始化输出变量
        v_out = v_in
        omega_out = omega_in
        
        # 2. 修正逻辑
        # 如果处于静止或极低速死区，直接给0
        if abs(v_in) < 0.01:
            v_out = 0.0
            omega_out = 0.0
            steering_angle = 0.0
        
        # 如果原始速度小于硬件限制 (且不为0)，触发修正
        elif abs(v_in) < self.min_speed:
            # 符号保持 (判断是前进还是后退)
            direction = 1.0 if v_in > 0 else -1.0
            
            # A. 强制提升速度
            v_out = self.min_speed * direction
            
            # B. 计算速度差的比率 (0.0 ~ 1.0)
            # ratio 越小，说明原始速度越慢，与 0.8 的差距越大
            # 例如：原始 0.2，强制 0.8 -> ratio = 0.25 (差距很大)
            # 例如：原始 0.7，强制 0.8 -> ratio = 0.875 (差距很小)
            ratio = abs(v_in) / self.min_speed
            
            # C. 应用角度修正函数
            # 逻辑：差距越大 (ratio越小)，我们需要把角速度 omega 缩得越小
            # 公式：omega_new = omega_old * (ratio ^ gain)
            scale_factor = math.pow(ratio, self.gain)
            
            omega_out = omega_in * scale_factor
            
            self.get_logger().debug(
                f'修正触发: 原始v={v_in:.2f}, 强制v={v_out:.2f}, '
                f'原始w={omega_in:.2f}, 修正系数={scale_factor:.2f}, 修正w={omega_out:.2f}'
            )
            
        else:
            # 速度大于 0.8，无需修正，直接透传
            v_out = v_in
            omega_out = omega_in

        # 3. 发布修正后的 cmd_vel_corrective
        corrected_twist = Twist()
        corrected_twist.linear.x = float(v_out)
        corrected_twist.angular.z = float(omega_out)
        self.pub_cmd_vel_corrective.publish(corrected_twist)

def main(args=None):
    rclpy.init(args=args)
    node = VelocityCorrectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()