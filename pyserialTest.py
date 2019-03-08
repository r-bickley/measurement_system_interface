import serial
import time
# pyserial installation files and instructions at:
# https://pyserial.readthedocs.io/en/latest/index.html

# Serial Setup
ser = serial.Serial('COM3', 9600)
ser.baudrate = 9600
ser.port = 'COM3'
print(ser.name)
print(ser.is_open)
time.sleep(1)

# Ask if <Arduino is ready>
ser.write(b'<-1,-1,-1>')
arduinoReady = ser.read();
time.sleep(5)
if arduinoReady == 1:
    print('Arduino is ready')
else:
    print('Arduino is not ready')

#if ser.is_open:
#    time.sleep(1)
#    ser.write(b'<0,1000,0>')
ser.close()
#print('done')
