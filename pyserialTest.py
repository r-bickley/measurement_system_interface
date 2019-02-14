import serial
import time
# pyserial installation files and instructions at:
# https://pyserial.readthedocs.io/en/latest/index.html

ser = serial.Serial('COM3', 9600)
ser.baudrate = 9600
ser.port = 'COM3'
print(ser.name)
print(ser.is_open)
if ser.is_open:
    t = 0
    while(t<3000000):
        #if (t % 10 == 0):
        #    print(t)
        t += 1
    ser.write(b'1')
ser.close()
print('done')
