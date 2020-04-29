#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from fiducial_msgs.msg import *
from wheel_control.msg import path_message
import sys, select, os
import math
import time

e = """
Communications Failed
"""

msg = """
Control Your TurtleBot3!
---------------------------
press r to draw a rectangle

space key, s : force stop

CTRL-C to quit
"""
def callback(data):
	global fids,ids
	fids=data.fiducials
	for i in range (len(fids)):
		fid=fids[i];
    		rospy.loginfo(rospy.get_caller_id() + "I saw %d", fid.fiducial_id)
		ids.append(fid.fiducial_id)
if __name__=="__main__":
	if os.name != 'nt':
		settings = termios.tcgetattr(sys.stdin)

	rospy.init_node('create_path')
	pub = rospy.Publisher('path', path_message, queue_size=10)
    	rospy.Subscriber("/fiducial_vertices", FiducialArray, callback)

	try:
		print(msg)
		while(1):

			if key == 'r' :
				drawRectangle(1,2,90)  
			elif key == 's':
				message = path_message()
				message.function = "stop"
				message.distance= 0.0
				message.angle = 0.0
				pub.publish(message)
			else:
				if (key == '\x03'):
					break

	except:
		print(e)

	finally:
		message = path_message()
		message.function = ""
		message.distance = 0.0
		message.angle = 0.0
		pub.publish(message)

	if os.name != 'nt':
		termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

def publish_message(func,dist,ang):
	message = path_message()
       	message.function = func
	message.distance = dist
	message.angle = ang
        pub.publish(message)

def drawRectangle(d1,d2,a): 
    publish_message(stop,0,0)   
    publish_message(straigth,d1,0)
    publish_message(stop,0,0)
    publish_message(turn,0,a)
    publish_message(straigth,d2,0)
    publish_message(stop,0,0)
    publish_message(turn,0,a)
    publish_message(straigth,d1,0)
    publish_message(stop,0,0)
    publish_message(turn,0,a)
    publish_message(straigth,d2,0)
    stop()
    return
