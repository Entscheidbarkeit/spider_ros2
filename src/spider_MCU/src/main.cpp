#include <rclcpp/rclcpp.hpp>
#include "spider_MCU/McuNode.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MCU::McuNode>());
  rclcpp::shutdown();
  return 0;
}