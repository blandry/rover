#pragma once

#include "std_msgs/String.h"


struct vel_setpoint_t {
    double vx;
    double vy;
    double vyaw;
};

struct wheel_state_t {
    double wheel_yaw[6];
    double wheel_speed[6];
};

struct wheel_command_t {
    double wheel_yaw_cmds[6];
    double wheel_speed_cmds[6];
};

struct mast_setpoint_t {
    unsigned int mast_pitch;
    unsigned int mast_yaw;
};


class Autopilot {

public:
    Autopilot();
    ~Autopilot();
    void task_main();
    void setpoint_to_command();
    void execute_command();
    void yaw_rate_to_command(double vyaw_sp);
    void vel_to_command(double vx_sp, double vy_sp);
    bool yaw_wheel(int wheel_num, double wheel_yaw_sp);
    unsigned int wheel_yaw_to_pwm(int wheel_num, double wheel_yaw);
    unsigned int wheel_speed_to_pwm(int wheel_num, double wheel_speed);
    void send_pwm(unsigned int servo_id, unsigned int value);
    void rover_cmds_callback(const std_msgs::String::ConstPtr& msg);

private:
    int ioboard;

    const double max_wheel_yaw_step_size = 5.0;
    const double yawing_wheel_pos[6] = {45.0, -45.0, 0.0, 45.0, -45.0, 0.0};
    const unsigned int wheel_zero_yaw_cmd[6] = {1500, 1400, 1415, 1240, 1330, 1325};
    const double wheel_yaw_slope[6] = {-11.1, -11.1, -11.1, -11.1, -11.1, -11.1};
    const unsigned int wheel_zero_speed_cmd[6] = {1400, 1400, 1390, 1390, 1375, 1400};
    const double wheel_speed_slope[6] = {-1.0, 1.0, 1.0, 1.0, -1.0, -1.0};
    const int left_wheels[3] = {1, 2, 3};
    const int right_wheels[3] = {0, 4, 5};

    vel_setpoint_t vel_setpoint;
    wheel_state_t wheel_state;
    wheel_command_t wheel_command;
    mast_setpoint_t mast_setpoint;

};

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}
