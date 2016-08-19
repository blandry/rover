from time import sleep
from arduino import ArduinoBoard

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
    
    arduino = ArduinoBoard('/dev/tty.usbmodem1421', baud_rate=115200)
    
    while True:
        
        pass


if __name__ == '__main__':
    main()