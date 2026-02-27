#include <rclcpp/rclcpp.hpp>
#include "ros2_orb_slam3/common.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<StereoModeNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
