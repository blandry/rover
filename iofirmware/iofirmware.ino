#include "config.h"
#include "CmdMessenger.h"
#include "Adafruit_PWMServoDriver.h"

enum {
  echo,
  servo,
};

// 1000000 / 60 / 4096
double pulselength = 4.06901041667;

Adafruit_PWMServoDriver pwm_out = Adafruit_PWMServoDriver();
CmdMessenger c = CmdMessenger(Serial,',',';','/');

void on_echo(void){

  int value = c.readBinArg<int>();
  c.sendBinCmd(echo,value);
  
}

void on_servo(void){

  unsigned int id = c.readBinArg<unsigned int>();
  unsigned int value = c.readBinArg<unsigned int>();

  pwm_out.setPWM(id, 0, ((double)value)/pulselength);
}

void attach_callbacks(void){
  c.attach(echo, on_echo);
  c.attach(servo, on_servo);
}

void setup() {
  Serial.begin(COMPANION_BAUD);
  pwm_out.begin();
  pwm_out.setPWMFreq(60);
  attach_callbacks();
}

void loop() {
  c.feedinSerialData();
}
