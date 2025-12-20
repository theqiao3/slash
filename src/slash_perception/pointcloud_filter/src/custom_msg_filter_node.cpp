/**
 * @file custom_msg_filter_node.cpp
 * @brief Livox CustomMsg点云半径过滤节点 (C++实现)
 * 
 * 订阅 /livox/lidar (CustomMsg类型)，过滤半径内的点云，重新发布到 /livox/lidar_filtered
 * 用于FAST_LIO等定位导航功能
 */

#include <rclcpp/rclcpp.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <livox_ros_driver2/msg/custom_point.hpp>
#include <cmath>
#include <vector>

class CustomMsgFilterNode : public rclcpp::Node
{
public:
  CustomMsgFilterNode() : Node("custom_msg_filter_node"), processed_count_(0)
  {
    // 声明参数
    this->declare_parameter<double>("min_radius", 0.2);
    this->declare_parameter<std::string>("input_topic", "/livox/lidar");
    this->declare_parameter<std::string>("output_topic", "/livox/lidar_filtered");
    
    // 获取参数
    this->get_parameter("min_radius", min_radius_);
    this->get_parameter("input_topic", input_topic_);
    this->get_parameter("output_topic", output_topic_);
    
    // 计算平方以避免重复开方运算
    min_radius_squared_ = min_radius_ * min_radius_;
    
    // 创建订阅者和发布者
    subscription_ = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(
      input_topic_, 10,
      std::bind(&CustomMsgFilterNode::customMsgCallback, this, std::placeholders::_1));
    
    publisher_ = this->create_publisher<livox_ros_driver2::msg::CustomMsg>(
      output_topic_, 10);
    
    RCLCPP_INFO(this->get_logger(), "Livox CustomMsg点云半径过滤节点已启动");
    RCLCPP_INFO(this->get_logger(), "订阅话题: %s", input_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "发布话题: %s", output_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "最小半径: %.2f 米", min_radius_);
  }

private:
  void customMsgCallback(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg)
  {
    if (msg->point_num == 0) {
      RCLCPP_WARN(this->get_logger(), "接收到空点云");
      return;
    }
    
    // 创建输出消息
    auto filtered_msg = std::make_shared<livox_ros_driver2::msg::CustomMsg>();
    filtered_msg->header = msg->header;
    filtered_msg->timebase = msg->timebase;
    filtered_msg->lidar_id = msg->lidar_id;
    filtered_msg->rsvd = msg->rsvd;
    
    // 预分配空间
    filtered_msg->points.reserve(msg->point_num);
    
    // 过滤点云
    size_t original_count = msg->point_num;
    size_t removed_count = 0;
    
    for (const auto& point : msg->points) {
      // 计算点到原点的距离平方
      double distance_squared = point.x * point.x + 
                               point.y * point.y + 
                               point.z * point.z;
      
      // 只保留距离大于等于min_radius的点
      if (distance_squared >= min_radius_squared_) {
        filtered_msg->points.push_back(point);
      } else {
        removed_count++;
      }
    }
    
    filtered_msg->point_num = filtered_msg->points.size();
    
    // 统计信息 (每100帧打印一次)
    processed_count_++;
    if (processed_count_ % 100 == 0) {
      RCLCPP_INFO(this->get_logger(), 
                  "处理第%ld帧: 原始点数=%zu, 过滤后=%u, 移除=%zu",
                  processed_count_, original_count, 
                  filtered_msg->point_num, removed_count);
    }
    
    // 发布过滤后的点云
    publisher_->publish(*filtered_msg);
  }
  
  // 成员变量
  rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr subscription_;
  rclcpp::Publisher<livox_ros_driver2::msg::CustomMsg>::SharedPtr publisher_;
  
  double min_radius_;
  double min_radius_squared_;
  std::string input_topic_;
  std::string output_topic_;
  size_t processed_count_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CustomMsgFilterNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
