#include "config.h"
#include "Adafruit_PWMServoDriver.h"
#include <ArduinoJson.h>

double pulselength = 4.06901041667; // 1000000 / 60 / 4096
Adafruit_PWMServoDriver pwm_out = Adafruit_PWMServoDriver();
String json_string;

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
      
        json_string = Serial.readString();          
        StaticJsonBuffer<200> json_buffer;
        JsonObject& msg = json_buffer.parseObject(json_string);
        
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
