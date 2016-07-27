import serial
import struct

def main():
    arduino = serial.Serial('/dev/tty.usbmodem1421', 9600)
    
    cmd = struct.pack('<BH',0,1600)
    while True:
        arduino.write(cmd)

if __name__ == '__main__':
    main()