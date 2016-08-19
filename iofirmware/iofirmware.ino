#include "config.h"
#include "Adafruit_PWMServoDriver.h"

Adafruit_PWMServoDriver pwm_out = Adafruit_PWMServoDriver();

void on_servo(int id, int value) {
    
    pwm_out.setPWM(id, 0, ((double)value)/PULSE_LENGTH);
}

void on_unknown() {
    
    // todo
}

void setup() {
    
    Serial.begin(COMPANION_BAUD);
    Serial.setTimeout(1000);
    pwm_out.begin();
    pwm_out.setPWMFreq(60);
}

void loop() {

    char receive_buf[100];    
    while(Serial.available()) {
        
        int num_bytes = Serial.readBytesUntil(';', receive_buf, sizeof(receive_buf)-1);
        receive_buf[num_bytes] = NULL;
        
        char *arg = strtok(receive_buf, ",;");

        // first is the command id
        int cmd_id = atoi(arg);

        int args[MAX_CMD_ARGS];
        int num_args = 0;
        for (int i=0; i<MAX_CMD_ARGS; i++) {
            arg = strtok(NULL, ",;");
            if (arg == NULL) {
                break;
            }
            args[i] = atoi(arg);
            num_args++;
        }

        // Serial.print(cmd_id);
        // Serial.print(" = ( ");
        // for (int i=0; i<num_args; i++) {
        //     Serial.print(args[i]);
        //     Serial.print(" ");
        // }
        // Serial.print(") ;");
        // Serial.flush();

        switch (cmd_id) {
            case 1:
                on_servo(args[0], args[1]);
                break;
            default:
                on_unknown();
                break;
        }
    }
}
