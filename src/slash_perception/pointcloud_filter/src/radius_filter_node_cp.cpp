/**
 * @file radius_filter_node.cpp
 * @brief PointCloud2点云半径过滤节点 (C++实现)
 * 
 * 订阅 /livox/lidar/pointcloud (PointCloud2类型)，过滤半径内的点云，
 * 重新发布到 /livox/lidar/pointcloud_filtered
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <cmath>

class RadiusFilterNode : public rclcpp::Node
{
public:
  RadiusFilterNode() : Node("radius_filter_node"), processed_count_(0)
  {
    // 声明参数
    this->declare_parameter<double>("min_radius", 0.2);
    this->declare_parameter<std::string>("input_topic", "/livox/lidar/pointcloud");
    this->declare_parameter<std::string>("output_topic", "/livox/lidar/pointcloud_filtered");
    
    // 获取参数
    this->get_parameter("min_radius", min_radius_);
    this->get_parameter("input_topic", input_topic_);
    this->get_parameter("output_topic", output_topic_);
    
    // 计算平方以避免重复开方运算
    min_radius_squared_ = min_radius_ * min_radius_;
    
    // 创建订阅者和发布者
    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, 10,
      std::bind(&RadiusFilterNode::pointCloudCallback, this, std::placeholders::_1));
    
    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      output_topic_, 10);
    
    RCLCPP_INFO(this->get_logger(), "PointCloud2点云半径过滤节点已启动");
    RCLCPP_INFO(this->get_logger(), "订阅话题: %s", input_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "发布话题: %s", output_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "最小半径: %.2f 米", min_radius_);
  }

private:
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    // 转换为PCL点云
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*msg, *cloud);
    
    if (cloud->empty()) {
      RCLCPP_WARN(this->get_logger(), "接收到空点云");
      return;
    }
    
    // 创建输出点云
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    filtered_cloud->header = cloud->header;
    filtered_cloud->is_dense = cloud->is_dense;
    
    // 预分配空间
    filtered_cloud->points.reserve(cloud->points.size());
    
    // 手动过滤点云
    size_t original_count = cloud->points.size();
    size_t removed_count = 0;
    
    for (const auto& point : cloud->points) {
      // 计算点到原点的距离平方
      double distance_squared = point.x * point.x + 
                               point.y * point.y + 
                               point.z * point.z;
      
      // 只保留距离大于等于min_radius的点
      if (distance_squared >= min_radius_squared_) {
        filtered_cloud->points.push_back(point);
      } else {
        removed_count++;
      }
    }
    
    filtered_cloud->width = filtered_cloud->points.size();
    filtered_cloud->height = 1;
    
    // 统计信息 (每100帧打印一次)
    processed_count_++;
    if (processed_count_ % 100 == 0) {
      RCLCPP_INFO(this->get_logger(), 
                  "处理第%ld帧: 原始点数=%zu, 过滤后=%zu, 移除=%zu",
                  processed_count_, original_count, 
                  filtered_cloud->points.size(), removed_count);
    }
    
    // 转换回ROS消息并发布
    sensor_msgs::msg::PointCloud2 output_msg;
    pcl::toROSMsg(*filtered_cloud, output_msg);
    output_msg.header = msg->header;
    
    publisher_->publish(output_msg);
  }
  
  // 成员变量
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  
  double min_radius_;
  double min_radius_squared_;
  std::string input_topic_;
  std::string output_topic_;
  size_t processed_count_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RadiusFilterNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
