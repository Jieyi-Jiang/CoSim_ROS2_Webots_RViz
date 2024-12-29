#include "my_package/MyRobotDriver.hpp"
#include "my_package/Kinematics.hpp"

#include "rclcpp/rclcpp.hpp"
#include <cstdio>
#include <functional>

// include typical webots header files
// #include <webots/Robot.hpp>
// #include <webots/Motor.hpp>
#include <webots/robot.h>
#include <webots/motor.h>

#include <tf2/utils.h>
#include <tf2_ros/transform_broadcaster.h>

#define HALF_DISTANCE_BETWEEN_WHEELS 0.045
#define WHEEL_RADIUS 0.025

// using namespace webots;
namespace my_robot
{

    // input the ros node and parameters, when the robot driver is created
    void MyRobotDriver::init(
        webots_ros2_driver::WebotsNode *node,
        std::unordered_map<std::string, std::string> &parameters)
    {
        robot_node = node;
        // robot = new Robot();
        // get the motors
        leftMotor = wb_robot_get_device("left wheel motor");
        rightMotor = wb_robot_get_device("right wheel motor");

        // set the motors to be controlled by velocity
        // set the position to infinity, so that the volocity control is enabled
        wb_motor_set_position(leftMotor, INFINITY);
        wb_motor_set_velocity(leftMotor, 0.0);
        wb_motor_set_position(rightMotor, INFINITY);
        wb_motor_set_velocity(rightMotor, 0.0);

        // create a subscription to the cmd_vel topic
        cmd_vel_subscription_ = node->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel",
            rclcpp::SensorDataQoS().reliable(),
            std::bind(&MyRobotDriver::cmdVelCallback, this, std::placeholders::_1));
        tf_odom = std::make_unique<tf2_ros::TransformBroadcaster>(robot_node);
        // robot_kin.start(wb_robot_get_time());
        robot_kin.start(wb_robot_get_basic_time_step());
    }

    void odom_pulisher(MyRobotDriver *driver)
    {
        driver->odom = driver->robot_kin.kin_get_odom();
        geometry_msgs::msg::TransformStamped transform;
        double seconds = driver->robot_node->now().seconds();
        transform.header.stamp = rclcpp::Time(static_cast<uint64_t>(seconds * 1e9));
        transform.header.frame_id = "odom";
        transform.child_frame_id = "base_link";

        transform.transform.translation.x = driver->odom.pos_position.x;
        transform.transform.translation.y = driver->odom.pos_position.y;
        transform.transform.translation.z = driver->odom.pos_position.z;
        geometry_msgs::msg::Quaternion quaternion_msg;
        tf2::Quaternion quaternion;
        quaternion.setRPY(driver->odom.pos_orientation.x, driver->odom.pos_orientation.y, driver->odom.pos_orientation.z);
        transform.transform.rotation.x = quaternion.x();
        transform.transform.rotation.y = quaternion.y();
        transform.transform.rotation.z = quaternion.z();
        transform.transform.rotation.w = quaternion.w();

        // 广播坐标变换信息
        driver->tf_odom->sendTransform(transform);
    }
    void footprint_pulisher(MyRobotDriver *driver)
    {
        geometry_msgs::msg::TransformStamped transform;
        double seconds = driver->robot_node->now().seconds();
        transform.header.stamp = rclcpp::Time(static_cast<uint64_t>(seconds * 1e9));
        transform.header.frame_id = "base_link";
        transform.child_frame_id = "base_footprint";
        transform.transform.translation.x = 0.0;
        transform.transform.translation.y = 0.0;
        transform.transform.translation.z = 0.0;
        transform.transform.rotation.x = 0.0;
        transform.transform.rotation.y = 0.0;
        transform.transform.rotation.z = 0.0;
        transform.transform.rotation.w = 1.0;
        driver->tf_odom->sendTransform(transform);
    }

    void map_to_odom_pulisher(MyRobotDriver *driver)
    {
        geometry_msgs::msg::TransformStamped transform;
        double seconds = driver->robot_node->now().seconds();
        transform.header.stamp = rclcpp::Time(static_cast<uint64_t>(seconds * 1e9));
        transform.header.frame_id = "map";
        transform.child_frame_id = "odom";
        transform.transform.translation.x = 0.0;
        transform.transform.translation.y = 0.0;
        transform.transform.translation.z = 0.0;
        transform.transform.rotation.x = 0.0;
        transform.transform.rotation.y = 0.0;
        transform.transform.rotation.z = 0.0;
        transform.transform.rotation.w = 1.0;
        driver->tf_odom->sendTransform(transform);
    }

    void MyRobotDriver::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        cmd_vel_msg.linear = msg->linear;
        cmd_vel_msg.angular = msg->angular;
    }

    void MyRobotDriver::step()
    {
        robot_kin.kin_forward(cmd_vel_msg.linear.x, cmd_vel_msg.angular.z);
        // auto forward_speed = cmd_vel_msg.linear.x;
        // auto angular_speed = cmd_vel_msg.angular.z;

        // // calculate the forward speed and angular speed of the robot
        // auto command_motor_left =
        //     (forward_speed - angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) /
        //     WHEEL_RADIUS;
        // auto command_motor_right =
        //     (forward_speed + angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) /
        //     WHEEL_RADIUS;
        /// set the velocity of the motors to move the robot
        wb_motor_set_velocity(leftMotor, robot_kin.kin_get_left_motor_speed());
        wb_motor_set_velocity(rightMotor, robot_kin.kin_get_right_motor_speed());
        robot_kin.kin_update_measure(wb_motor_get_velocity(leftMotor), wb_motor_get_velocity(rightMotor));
        robot_kin.kin_update_ticks(wb_robot_get_time());
        robot_kin.kin_updata_odom();
        odom_pulisher(this);
        static int publish_cnt = 0;
        if (publish_cnt >= 20)
        {
            footprint_pulisher(this);
            map_to_odom_pulisher(this);
            publish_cnt = 0;
        }
        publish_cnt += 1;
    }
} // namespace my_robot_driver

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(my_robot::MyRobotDriver,
                       webots_ros2_driver::PluginInterface)