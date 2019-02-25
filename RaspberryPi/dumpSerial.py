import serial
import sys
if len(sys.argv)==2:
    bauds = int(sys.argv[1])
else:
    bauds = 9600
ser = serial.Serial('/dev/ttyUSB0', bauds)
while True:
    sys.stdout.write(ser.read())
