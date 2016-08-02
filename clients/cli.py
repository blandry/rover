from time import sleep
from arduino import ArduinoBoard
from PyCmdMessenger import CmdMessenger

# y0 = 380
# y1 = 350
# y2 = 340
# y3 = 300
# y4 = 360
# y5 = 350

y0 = 1500
y1 = 1420 - 20
y2 = 1465 - 50
y3 = 1300 - 60
y4 = 1575 - 30
y5 = 1460 - 20

s0 = 1725 - 50
s1 = 1715 - 75
s2 = 1900 - 70
s3 = 1380 - 60
s4 = 1840 - 60
s5 = 1750 - 70

def main():
    
    arduino = ArduinoBoard('/dev/tty.usbmodem1411', baud_rate=9600)
    
    commands = [
        ["echo","i"],
        ["servo","II"],
    ]
    
    c = CmdMessenger(arduino,commands)
    
    speed = 50
    
    while True:
        
        c.send("servo", 0, s0 - speed)
        c.send("servo", 1, s1 + speed)
        c.send("servo", 2, s2 + speed)
        c.send("servo", 3, s3 + speed)
        c.send("servo", 4, s4 - speed)
        c.send("servo", 5, s5 - speed)
        
        c.send("servo", 6, y0)
        c.send("servo", 7, y1)
        c.send("servo", 8, y2)
        c.send("servo", 9, y3)
        c.send("servo", 10, y4)
        c.send("servo", 11, y5)
 

if __name__ == '__main__':
    main()