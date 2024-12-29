/**
 * Kinematics class for two wheels differential control robot.
 */

#pragma once
#ifndef KINEMATICS_HPP
#define KINEMATICS_HPP

#include <sys/types.h>

namespace my_robot
{

struct Vector3
{
    float x;
    float y;
    float z;
};

struct Odom_t
{
    Vector3 pos_position;
    Vector3 pos_orientation;
    Vector3 vol_angular;
    Vector3 vol_linear;
};

struct Robot_params_t
{
    float wheel_distance;       // distance of two wheels
    float wheel_radius;         // radius of wheel
};

struct Robot_motor_t
{
    float left_motor_speed_m;   // rad/s, measured value
    float right_motor_speed_m;  // rad/s, measured value
    float left_motor_speed_t;   // rad/s, traget value
    float right_motor_speed_t;  // rad/s, traget value
};


class Kinematics{

public:
    explicit Kinematics();
    Kinematics(float wheel_distance, float wheel_radius);
    ~Kinematics();
    void kin_forward(float vol_l_x, float vol_a_z);
    void kin_inverse();
    void kin_update_ticks(u_int64_t time_ns) { this->time_now_ = time_ns; }
    void kin_updata_odom();
    Odom_t &kin_get_odom() { return odom_; }
    float kin_get_left_motor_speed() { return robot_motor_.left_motor_speed_t; }
    float kin_get_right_motor_speed() { return robot_motor_.right_motor_speed_t; }
    void kin_update_measure(float left, float right) { 
        robot_motor_.left_motor_speed_m = left;
        robot_motor_.right_motor_speed_m = right;}
    void start(double time){ base_time_ = time; }
    // void start(double time){ time_last_ = time; }
private:
    Odom_t odom_;
    double time_last_; 
    double time_now_; 
    Robot_params_t robot_params_;
    Robot_motor_t robot_motor_;
    void init_odom_();
    void init_robot_motor_();
    float base_time_;
};
}

#endif // KINEMATICS_HPP