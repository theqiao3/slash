#include "icp_localization/icp_localization.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<icp_localization::ICPLocalization>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
