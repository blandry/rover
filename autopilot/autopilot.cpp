#include "autopilot.h"
#include <algorithm>

Autopilot::Autopilot() {
  
}

void Autopilot::task_main() {
    
}

/* 
converts a velocity setpoint to an actuator command
to be execute until the next iteration
*/
void Autopilot::setpoint_to_command() {
    
    if vel_sepoint.v_yaw > 0 {
    // if a yaw rate is set, execute that
        for (int i=0; i<6; i++) {
            // first place the wheels in the right yaw
            if (wheel_state.wheel_yaw[i] != wheel_command.wheel_yaw_cmds[i]) {
                uint16_t delta = yawing_wheel_pos[i] - wheel_state.wheel_yaw[i];
                uint16_t cmd_delta_mag = std::min(max_servo_step_size, abs(delta));
                wheel_command.wheel_yaw_cmds[i] = sgn(delta) * cmd_delta_mag;
                wheel_command.wheel_speed_cmds[i] = wheel_zero_vel_cmd[i];
            } else {
                // todo: wheel speeds when yawing
                // wheel_command.wheel_speed_cmds[i] = 
            }
        }
    } else {
    // else execute the linear velocity (body frame)
        for (int i=0; i<6; i++) {
            // first place the wheels in the right yaw
            if (wheel_state.wheel_yaw[i] != wheel_command.wheel_yaw_cmds[i]) {
                uint16_t delta = yawing_wheel_pos[i] - ;
                uint16_t cmd_delta_mag = std::min(max_servo_step_size, abs(delta));
                wheel_command.wheel_yaw_cmds[i] = sgn(delta) * cmd_delta_mag;
                wheel_command.wheel_speed_cmds[i] = wheel_zero_vel_cmd[i];
            } else {
                // todo: wheel speeds when yawing
                // wheel_command.wheel_speed_cmds[i] = 
            }
        }
    }
    
}

void Autopilot::execute_command() {
    // sends the command to the IO board
    
    // assumes the command is executed instantenously (set the state as such)
}

int main(int argc, char *argv[]) {
  
    Autopilot autopilot = Autopilot();
    autopilot.task_main();
    
    return 0;
}