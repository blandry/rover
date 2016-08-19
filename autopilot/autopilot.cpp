#include "autopilot.h"
#include <algorithm>
#include <unistd.h>
#include <math.h>
#include <stdlib.h>
#include <iostream>
#include "arduino-serial-lib.h"
#include "ros/ros.h"
#include "std_msgs/String.h"

double rad_to_degrees(double x);

Autopilot::Autopilot() {
    
    ioboard = serialport_init("/dev/tty.usbmodem1421", 115200);

    // make sure the setpoints are initialized
    vel_setpoint.vx = 0.0;
    vel_setpoint.vy = 0.0;
    vel_setpoint.vyaw = 0.0;
}

Autopilot::~Autopilot() {
    
    serialport_close(ioboard);
}

void Autopilot::task_main() {

    while (true) {
        // gets all the new ROS messages
        ros::spinOnce();
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
    if (fabs(vel_setpoint.vyaw) > 0) {
        yaw_rate_to_command(vel_setpoint.vyaw);
    } else {
        vel_to_command(vel_setpoint.vx, vel_setpoint.vy);
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
    
    // compute the setpoints
    double wheel_yaw_sp = rad_to_degrees(atan2(vy_sp, vx_sp));
    double wheel_speed_sp = sqrt(pow(vx_sp, 2) + pow(vy_sp, 2));

    // don't rotate the wheel more than 180, spin the wheels backward
    if (wheel_yaw_sp < -90.0) {
        wheel_yaw_sp += 180.0;
        wheel_speed_sp *= -1.0;
    } else if (wheel_yaw_sp > 90.0) {
        wheel_yaw_sp -= 180.0;
        wheel_speed_sp *= -1.0;
    }
    
    ROS_INFO("vx=%f vy=%f yaw=%f spd=%f", vx_sp, vy_sp, wheel_yaw_sp, wheel_speed_sp);
    
    // sets the commands
    bool wheels_ready = true;
    for (int i=0; i<6; i++) {
        // first place the wheels in the right yaw
        wheel_command.wheel_speed_cmds[i] = 0;
        wheels_ready = (yaw_wheel(i, wheel_yaw_sp) && wheels_ready);
    }
    
    // if all the wheels are ready (in the right orientation)
    if (wheels_ready) {
        for (int i=0; i<6; i++) {
            wheel_command.wheel_speed_cmds[i] = wheel_speed_sp;
        }
    }
}

/* converts a rover angular velocity to wheel commands */
void Autopilot::yaw_rate_to_command(double vyaw_sp) {
    
    bool wheels_ready = true;
    for (int i=0; i<6; i++) {
        // first place the wheels in the right yaw
        wheel_command.wheel_speed_cmds[i] = 0;
        wheels_ready = (yaw_wheel(i, yawing_wheel_pos[i]) && wheels_ready);
    }
    
    // if all the wheels are ready (in the right orientation)
    if (wheels_ready) {
        for (int i=0; i<3; i++) {
            wheel_command.wheel_speed_cmds[right_wheels[i]] = vyaw_sp;
        }
        for (int i=0; i<3; i++) {
            wheel_command.wheel_speed_cmds[left_wheels[i]] = -1.0 * vyaw_sp;
        }
    }
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
    
    unsigned int pwm = (unsigned int)(wheel_yaw * wheel_yaw_slope[wheel_num]) + wheel_zero_yaw_cmd[wheel_num];
    
    return pwm;
}

unsigned int Autopilot::wheel_speed_to_pwm(int wheel_num, double wheel_speed) {
    
    unsigned int pwm = (unsigned int)(wheel_speed * wheel_speed_slope[wheel_num]) + wheel_zero_speed_cmd[wheel_num];
    
    return pwm;
}

void Autopilot::send_pwm(unsigned int servo_id, unsigned int value) {
    
    char send_buf[100];
    
    send_buf[0] = '1'; // the command id corresponding to servo cmds
    send_buf[1] = ',';
    
    char servo_id_char[100];
    sprintf(servo_id_char, "%i", servo_id);
    memcpy(send_buf + 2, servo_id_char, strlen(servo_id_char)); // removing trailing NULL character

    send_buf[2 + strlen(servo_id_char)] = ',';
    
    char value_char[100];
    sprintf(value_char, "%i", value);
    memcpy(send_buf + 2 + strlen(servo_id_char) + 1, value_char, strlen(value_char)); // removing trailing NULL character
    
    send_buf[2 + strlen(servo_id_char) + 1 + strlen(value_char)] = ';';

    // send_buf[2 + strlen(servo_id_char) + 1 + strlen(value_char) + 1] = NULL;
    // std::cout << "CMD: " << send_buf << std::endl;
    
    for (int i=0; i<(2+strlen(servo_id_char)+1+strlen(value_char)+1); i++) {
        serialport_writebyte(ioboard, send_buf[i]);
    }
    serialport_flush(ioboard);

    // char receive_buf[100];
    // serialport_read_until(ioboard, receive_buf, ';', sizeof(receive_buf), 1000);
    // std::cout << "ACK: " << receive_buf << std::endl;
}

void Autopilot::rover_cmds_callback(const std_msgs::String::ConstPtr& msg) {
    
    const char *cmd = msg->data.c_str();
    ROS_INFO("ROS CMD: [%s]", cmd);
    
    double default_speed = 50.0;
    double default_yawing_speed = 25.0;
    
    if (strcmp(cmd,"RIGHT")==0) {
        vel_setpoint.vx = 0.0;
        vel_setpoint.vy = -1.0 * default_speed;
        vel_setpoint.vyaw = 0.0;
    } else if (strcmp(cmd,"LEFT")==0) {
        vel_setpoint.vx = 0.0;
        vel_setpoint.vy = default_speed;
        vel_setpoint.vyaw = 0.0;
    } else if (strcmp(cmd,"FORWARD")==0) {
        vel_setpoint.vx = default_speed;
        vel_setpoint.vy = 0.0;
        vel_setpoint.vyaw = 0.0;
    } else if (strcmp(cmd,"BACKWARD")==0) {
        vel_setpoint.vx = -1.0 * default_speed;
        vel_setpoint.vy = 0.0;
        vel_setpoint.vyaw = 0.0;
    } else if (strcmp(cmd,"NONE")==0) {
        vel_setpoint.vx = 0.0;
        vel_setpoint.vy = 0.0;
        vel_setpoint.vyaw = 0.0;
    } else if (strcmp(cmd,"YAWLEFT")==0) {
        vel_setpoint.vx = 0.0;
        vel_setpoint.vy = 0.0;
        vel_setpoint.vyaw = default_yawing_speed;
    } else if (strcmp(cmd,"YAWLEFT")==0) {
        vel_setpoint.vx = 0.0;
        vel_setpoint.vy = 0.0;
        vel_setpoint.vyaw = -1.0 * default_yawing_speed;    
    }
}

double rad_to_degrees(double x) {
    return x * 360 / (2*M_PI);
}

int main(int argc, char *argv[]) {

    Autopilot autopilot = Autopilot();
    
    ros::init(argc, argv, "Autopilot");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("rover_cmds", 1000, &Autopilot::rover_cmds_callback, &autopilot);
  
    autopilot.task_main();
    
    return 0;
}