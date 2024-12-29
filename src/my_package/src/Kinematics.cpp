#include "Kinematics.hpp"
#include <cmath>
#include <cstdio>

namespace my_robot
{
    Kinematics::Kinematics()
    {
        robot_params_.wheel_distance  = 0.0;
        robot_params_.wheel_radius = 0.0;
        init_odom_();
        init_robot_motor_();
        time_last_ = 0;
        time_now_ = 0;
    }

    Kinematics::Kinematics(float wheel_distance, float wheel_radius)
    {
        robot_params_.wheel_distance  = wheel_distance;
        robot_params_.wheel_radius = wheel_radius;
        init_odom_();
        init_robot_motor_();
        time_last_ = 0;
        time_now_ = 0;
    }

    Kinematics::~Kinematics() {};

    void Kinematics::kin_forward(float vol_l_x, float vol_a_z)
    {
        robot_motor_.left_motor_speed_t = (vol_l_x - vol_a_z*robot_params_.wheel_distance/2) / robot_params_.wheel_radius;
        robot_motor_.right_motor_speed_t = (vol_l_x + vol_a_z*robot_params_.wheel_distance/2) / robot_params_.wheel_radius;
    }

    void Kinematics::kin_inverse()
    {
        odom_.vol_linear.x = (robot_params_.wheel_radius/2.0) * (robot_motor_.left_motor_speed_m + robot_motor_.right_motor_speed_m);
        odom_.vol_angular.z = (robot_params_.wheel_radius/robot_params_.wheel_distance) * (-robot_motor_.left_motor_speed_m + robot_motor_.right_motor_speed_m);
        printf("vol_linea: %f\n", odom_.vol_linear.x);
        printf("vol_angular: %f\n", odom_.vol_angular.z);
    }


    void Kinematics::kin_updata_odom()
    {
        kin_inverse();
        odom_.vol_linear.y  = 0;
        odom_.vol_linear.z  = 0;
        odom_.vol_angular.x = 0;
        odom_.vol_angular.y = 0;

        // float dt = (time_now_ - time_last_);
        float dt = base_time_ * 1e-3;
        printf("dt: %f\n", dt);
        float d_distance = odom_.vol_linear.x * dt;
        float d_angle = odom_.vol_angular.z * dt;
        printf("d_distance: %f\n", d_distance);
        printf("d_angle: %f\n", d_angle);
        printf("cos: %f\n", cosf32(odom_.pos_orientation.z));
        printf("sin: %f\n", sinf32(odom_.pos_orientation.z));
        odom_.pos_position.x += d_distance * cosf32(odom_.pos_orientation.z);
        odom_.pos_position.y += d_distance * sinf32(odom_.pos_orientation.z);
        odom_.pos_position.z    = 0.0;
        odom_.pos_orientation.x = 0.0;
        odom_.pos_orientation.y = 0.0;
        odom_.pos_orientation.z += d_angle;
        time_last_ = time_now_;
        // printf("time_now_: %f\n", time_now_);
        // printf("d_time: %f\n", dt);

        printf("x: %f\n", odom_.pos_position.x);
        printf("angle: %f\n", odom_.pos_orientation.z);

    }

    void Kinematics::init_odom_()
    {
        odom_.pos_orientation.x = 0.0;
        odom_.pos_orientation.y = 0.0;
        odom_.pos_orientation.z = 0.0;
        odom_.pos_position.x    = 0.0;
        odom_.pos_position.y    = 0.0;
        odom_.pos_position.z    = 0.0;
        odom_.vol_linear.x      = 0.0;
        odom_.vol_linear.y      = 0.0;
        odom_.vol_linear.z      = 0.0;
        odom_.vol_angular.x     = 0.0;
        odom_.vol_angular.y     = 0.0;
        odom_.vol_angular.z     = 0.0;
    }

    void Kinematics::init_robot_motor_()
    {
        robot_motor_.left_motor_speed_m     = 0.0;
        robot_motor_.right_motor_speed_m    = 0.0;
        robot_motor_.left_motor_speed_t     = 0.0;
        robot_motor_.right_motor_speed_t    = 0.0;
    }

} // namespace my_robot
