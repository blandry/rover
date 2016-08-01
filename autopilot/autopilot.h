#pragma once

struct vel_setpoint_t {
    float vx;
    float vy;
    float vyaw;
}

struct wheel_state_t {
    uint16_t wheel_yaw[6];
    uint16_t wheel_speed[6];
}

struct wheel_command_t {
    uint16_t wheel_yaw_cmds[6];
    uint16_t wheel_speed_cmds[6];
}

class Autopilot {

public:
    Autopilot();
    void task_main();
    void setpoint_to_command();
    void execute_command();
    
    const uint16_t max_servo_step_size = 5;
    const uint16_t yawing_wheel_pos = {100, 100, 100, 100, 100, 100};
    const uint16_t wheel_zero_vel_cmd = {150, 150, 150, 150, 150, 150};
    
    vel_setpoint_t vel_sepoint;
    wheel_state_t wheel_state;
    wheel_command_t wheel_command;
    
};

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}