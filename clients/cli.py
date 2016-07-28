from time import sleep
from arduino import ArduinoBoard
from PyCmdMessenger import CmdMessenger

# PWM_MIN = 150
# PWM_MAX = 600
PWM_MIN = 225
PWM_MAX = 275

def main():
    
    arduino = ArduinoBoard('/dev/tty.usbmodem1421', baud_rate=9600)
    
    commands = [
        ["echo","i"],
        ["servo","II"],
    ]
    
    c = CmdMessenger(arduino,commands)
        
    pwm_value = PWM_MIN
    delta = 1
    while True:

        print(pwm_value)
        
        c.send("servo", 0, 600-pwm_value)
        c.send("servo", 1, pwm_value)
        c.send("servo", 2, 600-pwm_value)
        c.send("servo", 3, pwm_value)
        c.send("servo", 4, 600-pwm_value)
        c.send("servo", 5, pwm_value)
        
        # c.send("servo", 6, pwm_value)
        # c.send("servo", 7, pwm_value)
        # c.send("servo", 8, pwm_value)
        # c.send("servo", 9, pwm_value)
        # c.send("servo", 10, pwm_value)
        # c.send("servo", 11, pwm_value)

        if pwm_value >= PWM_MAX:
            delta = -5
        elif pwm_value <= PWM_MIN:
            delta = 5
        pwm_value += delta
        
        # sleep(.0001)

if __name__ == '__main__':
    main()