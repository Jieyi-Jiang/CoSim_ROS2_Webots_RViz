#ifndef WEBOTS_ROS2_PLUGIN_EXAMPLE_HPP
#define WEBOTS_ROS2_PLUGIN_EXAMPLE_HPP

#include "my_package/Kinematics.hpp"
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
#include "nav_msgs/msg/odometry.hpp"

#include <tf2/utils.h>
#include <tf2_ros/transform_broadcaster.h>

namespace my_robot
{
    class MyRobotDriver : public webots_ros2_driver::PluginInterface
    {
    public:
        void step() override;
        void init(webots_ros2_driver::WebotsNode *node,
                  std::unordered_map<std::string, std::string> &parameters) override;
        friend void odom_pulisher(MyRobotDriver *driver);
        friend void footprint_pulisher(MyRobotDriver *driver);
        friend void map_to_odom_pulisher(MyRobotDriver *driver);

    private:
        /// @brief Callback function for cmd_vel topic
        /// @param msg the message of cmd_vel topic
        void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr
            cmd_vel_subscription_;
        geometry_msgs::msg::Twist cmd_vel_msg;
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_odom;
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_base_footprint;
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_map_to_odom;

        // Robot *robot;
        WbDeviceTag leftMotor;
        WbDeviceTag rightMotor;
        Kinematics robot_kin = Kinematics(0.09, 0.025);
        Odom_t odom;
        webots_ros2_driver::WebotsNode *robot_node;
    };
} // namespace my_robot_driver
#endif