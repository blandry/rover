#pragma once

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

class Autopilot {

public:
    Autopilot();
    void task_main();
    void setpoint_to_command();
    void execute_command();
    void yaw_rate_to_command(double vyaw_sp);
    void vel_to_command(double vx_sp, double vy_sp);
    bool yaw_wheel(int wheel_num, double wheel_yaw_sp);
    unsigned int wheel_yaw_to_pwm(int wheel_num, double wheel_yaw);
    unsigned int wheel_speed_to_pwm(int wheel_num, double wheel_speed);
    void send_pwm(unsigned int servo_id, unsigned int value);
    
    const double max_wheel_yaw_step_size = 5.0;
    const double yawing_wheel_pos[6] = {45.0, -45.0, 0.0, 45.0, -45.0, 0.0};
    const unsigned int wheel_zero_yaw_cmd[6] = {1500, 1500, 1500, 1500, 1500, 1500};
    const double wheel_yaw_slope[6] = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
    const unsigned int wheel_zero_speed_cmd[6] = {1500, 1500, 1500, 1500, 1500, 1500};
    const double wheel_speed_slope[6] = {1.0, -1.0, -1.0, -1.0, 1.0, 1.0};
    
    vel_setpoint_t vel_sepoint;
    wheel_state_t wheel_state;
    wheel_command_t wheel_command;
    
};

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}