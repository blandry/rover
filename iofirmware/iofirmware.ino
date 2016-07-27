#include "config.h"
#include "messages.h"
#include "Adafruit_PWMServoDriver.h"

char actuator_cmd_buff[sizeof(actuator_cmd_t)];
actuator_cmd_t actuator_cmd;
Adafruit_PWMServoDriver pwm_out;

void setup() {
    Serial.begin(COMPANION_BAUD);
}

void loop() {
    if (Serial.available() >= sizeof(actuator_cmd_t)) {
        Serial.readBytes(actuator_cmd_buff, sizeof(actuator_cmd_t));
        memcpy(&actuator_cmd, actuator_cmd_buff, sizeof(actuator_cmd_t));
        
        pwm_out.setPin(actuator_cmd.id, actuator_cmd.value);
    }
}
