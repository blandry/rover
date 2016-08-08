#include "config.h"
#include "Adafruit_PWMServoDriver.h"
#include <ArduinoJson.h>


// 1000000 / 60 / 4096
double pulselength = 4.06901041667;

Adafruit_PWMServoDriver pwm_out = Adafruit_PWMServoDriver();
StaticJsonBuffer<200> json_buffer;
String serial_buffer;

void on_servo(JsonObject& msg){
    
    int id = msg["id"];
    int value = msg["value"];
    
    pwm_out.setPWM(id, 0, ((double)value)/pulselength);
}

void on_unknown(JsonObject& msg){
    
    // todo
}

void setup() {
    
    Serial.begin(COMPANION_BAUD);
    pwm_out.begin();
    pwm_out.setPWMFreq(60);
}

void loop() {
    
    while(Serial.available()) {
        serial_buffer = Serial.readString();
        JsonObject& msg = json_buffer.parseObject(serial_buffer);
        int cmd_id = msg["cmd"];
        
        switch (cmd_id) {
            case 1:
                on_servo(msg);
                break;
            default:
                on_unknown(msg);
        }
    }
}
