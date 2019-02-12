import serial
# pyserial installation files and instructions at:
# https://pyserial.readthedocs.io/en/latest/index.html

ser = serial.Serial()
ser.baudrate = 9600
ser.port = 'COM3'
print(ser.name)
print(ser.is_open)
if ser.is_open:
    ser.write(b'hello')
ser.close()
