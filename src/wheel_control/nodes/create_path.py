#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from wheel_control.msg import path_message
import sys, select, os
import math
import time
if os.name == 'nt':
  import msvcrt
else:
  import tty, termios

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
def getKey():
    if os.name == 'nt':
      return msvcrt.getch()

    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def publish_message(func,dist,ang):
	print("Hier geraakt.")
	message = path_message()
       	message.function = func
	message.distance = dist
	message.angle = ang
	print("message-function: " , str(message.function) , "message-distance: " , str(message.distance), "message-angle: " , str(message.angle))
        pub.publish(message)

def drawRectangle(d1,d2,a): 
    print("pulbiceren")
    publish_message("stop",0,0)   
    publish_message("straigth",d1,0)
    publish_message("stop",0,0)
    publish_message("turn",0,a)
    publish_message("straigth",d2,0)
    publish_message("stop",0,0)
    publish_message("turn",0,a)
    publish_message("straigth",d1,0)
    publish_message("stop",0,0)
    publish_message("turn",0,a)
    publish_message("straigth",d2,0)
    publish_message("stop",0,0)
    return

def drawTriangle(d1,a): 
    print("pulbiceren")
    publish_message("stop",0,0)   
    publish_message("straigth",d1,0)
    publish_message("stop",0,0)
    publish_message("turn",0,a)
    publish_message("straigth",d1,0)
    publish_message("stop",0,0)
    publish_message("turn",0,a)
    publish_message("straigth",d1,0)
    publish_message("stop",0,0)
    return

if __name__=="__main__":
	if os.name != 'nt':
		settings = termios.tcgetattr(sys.stdin)

	rospy.init_node('create_path')
	pub = rospy.Publisher('path', path_message, queue_size=10)

	try:
		
		print(msg)
		while(1):
			print("In lus")
			key = getKey()
			print(key)
			if key == 'r' :
				print("teken rechthoek")
				drawRectangle(1,2,90)  
			elif key == 'd' :
				print("teken driehoek")
				drawTriangle(2,120) 
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


