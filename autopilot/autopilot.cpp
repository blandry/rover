#include "autopilot.h"
#include <algorithm>
#include <unistd.h>
#include <math.h>
#include <stdlib.h>

double rad_to_degrees(double x);


Autopilot::Autopilot() {
  
}

void Autopilot::task_main() {
    
    while (true) {
        // converts the setpoint to a commands
        setpoint_to_command();
        // sends the command to the IO board
        execute_command();        
        // 10Hz
        usleep(100000);
    }
}

/* converts the setpoint to a commands */
void Autopilot::setpoint_to_command() {
    
    // prioritize yawing over moving
    if (vel_sepoint.vyaw > 0) {
        yaw_rate_to_command(vel_sepoint.vyaw);
    } else {
        vel_to_command(vel_sepoint.vx, vel_sepoint.vy);
    }
}

void Autopilot::execute_command() {

    // sends the command to the IO board
    for (int i=0; i<6; i++) {
        send_pwm(i, wheel_speed_to_pwm(i, wheel_command.wheel_speed_cmds[i]));
        send_pwm(i+6, wheel_yaw_to_pwm(i, wheel_command.wheel_yaw_cmds[i]));

        // assumes the command is executed instantenously (set the state as such)
        wheel_state.wheel_speed[i] = wheel_command.wheel_speed_cmds[i];
        wheel_state.wheel_yaw[i] = wheel_command.wheel_yaw_cmds[i];
    }    
}

/* converts a linear velocity to actuator commands */
void Autopilot::vel_to_command(double vx_sp, double vy_sp) {
    
    // makes sure we aren't yawing when moving
    yaw_rate_to_command(0);
    
    // compute the setpoints
    double wheel_yaw_sp = rad_to_degrees(atan2(vy_sp, vx_sp));
    double wheel_speed_sp = sqrt(pow(vx_sp, 2) + pow(vy_sp, 2));
    
    // sets the commands
    bool wheels_ready = false;
    for (int i=0; i<6; i++) {
        // first place the wheels in the right yaw
        wheel_command.wheel_speed_cmds[i] = 0;
        wheels_ready &= yaw_wheel(i, wheel_yaw_sp);
    }
    
    if (wheels_ready) {
        for (int i=0; i<6; i++) {
            wheel_command.wheel_speed_cmds[i] = wheel_speed_sp;
        }        
    }
}

/* converts a rover angular velocity to wheel commands */
void Autopilot::yaw_rate_to_command(double vyaw_sp) {
    // 
    // // makes sure we aren't moving when yawing
    // vel_to_command(0, 0);
    // 
    // for (int i=0; i<6; i++) {
    //     // first place the wheel in the right yaw
    //     
    // }
}

/* Places a given wheel at an angle in degrees, returns true
if the wheel is in the requested position, false otherwise */
bool Autopilot::yaw_wheel(int wheel_num, double wheel_yaw_sp) {
    
    if (wheel_state.wheel_yaw[wheel_num] != wheel_yaw_sp) {
        double delta = wheel_yaw_sp - wheel_state.wheel_yaw[wheel_num];
        double cmd_delta_mag = std::min(max_wheel_yaw_step_size, fabs(delta));
        wheel_command.wheel_yaw_cmds[wheel_num] += sgn(delta) * cmd_delta_mag;
        return false;
    } else {
        return true;
    }
}

unsigned int Autopilot::wheel_yaw_to_pwm(int wheel_num, double wheel_yaw) {

    unsigned int pwm = (unsigned int)(wheel_yaw - wheel_zero_yaw_cmd[wheel_num]) * wheel_yaw_slope[wheel_num] + wheel_zero_yaw_cmd[wheel_num];
    
    return pwm;
}

unsigned int Autopilot::wheel_speed_to_pwm(int wheel_num, double wheel_speed) {
    
    unsigned int pwm = (unsigned int)(wheel_speed - wheel_zero_speed_cmd[wheel_num]) * wheel_speed_slope[wheel_num] + wheel_zero_speed_cmd[wheel_num];
    
    return pwm;
}

void Autopilot::send_pwm(unsigned int servo_id, unsigned int value) {
    
    
}

double rad_to_degrees(double x) {
    return (x > 0 ? x : (2*M_PI + x)) * 360 / (2*M_PI);
}

int main(int argc, char *argv[]) {
  
    Autopilot autopilot = Autopilot();
    autopilot.task_main();
    
    return 0;
}