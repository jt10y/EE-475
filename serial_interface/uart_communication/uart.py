import serial
import time

ser = serial.Serial('/dev/ttyS0', 115200, timeout=1)
while True:
    send = ser.readline()
    print(send)
    
    #time.sleep(1000)
