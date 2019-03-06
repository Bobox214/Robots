import serial
import sys

supportedDevNames = ('ttyUSB0','rfcomm0')
supportedBauds    = (9600,115200)
if len(sys.argv)!=3:
    print('usage: python dumpSerial.py devName bauds')
    sys.exit(1)

devName = sys.argv[1]
if devName not in supportedDevNames:
    print("Unsupported devName '%s'. Supported are '%s'"%(devName,"','".join(supportedDevNames)))
    sys.exit(2)

try:
    bauds = int(sys.argv[2])
except:
    print("Unsupported bauds '%s'. Expected an integer"%sys.argv[1])
    sys.Exit(3)

if bauds not in supportedBauds:
    print("Unsupported bauds '%s'. Supported are '%s'"%(bauds,"','".join(str(x) for x in supportedBauds)))
    sys.exit(4)

print("## Starting reading on '/dev/%s' at %d bauds"%(devName,bauds))
ser = serial.Serial('/dev/%s'%devName, bauds)
while True:
    sys.stdout.write(ser.read())
    sys.stdout.flush()
