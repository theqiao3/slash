#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("set_initial_pose_node");
  
  auto publisher = node->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/initialpose", 1);
    
  // Wait for subscribers
  rclcpp::sleep_for(std::chrono::seconds(2));
  
  if (argc >= 4) {
    double x = std::stod(argv[1]);
    double y = std::stod(argv[2]);
    double yaw = std::stod(argv[3]);
    
    geometry_msgs::msg::PoseWithCovarianceStamped msg;
    msg.header.stamp = node->now();
    msg.header.frame_id = "map";
    
    msg.pose.pose.position.x = x;
    msg.pose.pose.position.y = y;
    msg.pose.pose.position.z = 0.0;
    
    // Yaw to quaternion
    msg.pose.pose.orientation.z = sin(yaw / 2.0);
    msg.pose.pose.orientation.w = cos(yaw / 2.0);
    
    // Covariance (simplified)
    msg.pose.covariance[0] = 0.25;
    msg.pose.covariance[7] = 0.25;
    msg.pose.covariance[35] = 0.0685;
    
    publisher->publish(msg);
    RCLCPP_INFO(node->get_logger(), "Published initial pose: x=%.2f, y=%.2f, yaw=%.2f", x, y, yaw);
  } else {
    RCLCPP_ERROR(node->get_logger(), "Usage: set_initial_pose_node <x> <y> <yaw>");
  }
  
  rclcpp::shutdown();
  return 0;
}
