#ifndef WEBOTS_ROS2_PLUGIN_EXAMPLE_HPP
#define WEBOTS_ROS2_PLUGIN_EXAMPLE_HPP

// macros of ros-rclcpp
#include "rclcpp/macros.hpp"
// plugin interface of webots-ros2
#include "webots_ros2_driver/PluginInterface.hpp"
// webots node api
#include "webots_ros2_driver/WebotsNode.hpp"
// ros geometry msg api
#include "geometry_msgs/msg/twist.hpp"
// ros client library
#include "rclcpp/rclcpp.hpp"

namespace my_robot_driver {
class MyRobotDriver : public webots_ros2_driver::PluginInterface {
public:
  void step() override;
  void init(webots_ros2_driver::WebotsNode *node,
            std::unordered_map<std::string, std::string> &parameters) override;

private:
  /// @brief Callback function for cmd_vel topic
  /// @param msg the message of cmd_vel topic
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr
      cmd_vel_subscription_;
  geometry_msgs::msg::Twist cmd_vel_msg;

  WbDeviceTag right_motor;
  WbDeviceTag left_motor;
};
} // namespace my_robot_driver
#endif