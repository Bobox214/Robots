#!/usr/bin/env python
import roslib; roslib.load_manifest('bluetooth_controller')
import rospy
import sys,time,signal,serial
import sys

from geometry_msgs.msg import Twist

def sigHandler(_signo, _stack_frame):
    sys.exit(1)

signal.signal(signal.SIGINT , sigHandler)
signal.signal(signal.SIGTERM, sigHandler)


btPort  = '/dev/rfcomm0'
btBauds = 115200

def interpret(ser):
    received = ''
    while True:
        received += ser.read()
        if 'mov:' in received:
            c = ser.read()
            return c

keyMaps = {
    'r' : ( 1, 1)
,   't' : ( 1, 0)
,   'y' : ( 1,-1)
,   'f' : ( 0, 1)
,   'g' : ( 0, 0)
,   'h' : ( 0,-1)
,   'v' : (-1, 1)
,   'b' : (-1, 0)
,   'n' : (-1,-1)
}
speed = 0.5
turn  = 1.0

def send_cmd_vel(c):
    x,w = keyMaps.get(c,(0,0))
    twist = Twist()
    twist.linear.x = x*speed
    twist.linear.y = 0
    twist.linear.z = 0
    twist.angular.x = 0
    twist.angular.y = 0
    twist.angular.z = w*turn
    pub.publish(twist)

if __name__ == '__main__':
    try:
        pub = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
        rospy.init_node('teleop_twist_bt')
        
        rospy.loginfo("Waiting for bluetooth connection through %s",btPort)
        connected = False
        while not rospy.is_shutdown():
            try:
                with serial.Serial(btPort,btBauds) as ser:
                    rospy.loginfo("Bluetooth connection done.")
                    connected = True
                    while True:
                        c = interpret(ser)
                        send_cmd_vel(c)
            except serial.serialutil.SerialException:
                 if connected:
                     rospy.logwarn("Bluetooth connection lost through %s",btPort)
                 connected = False
                 time.sleep(1)
                 continue
    except rospy.ROSInterruptException: pass
