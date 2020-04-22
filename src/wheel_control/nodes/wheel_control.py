#!/usr/bin/env python

# Copyright (c) 2011, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name of the Willow Garage, Inc. nor the names of its
#      contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import rospy
from geometry_msgs.msg import Twist
import path_message.msg
import sys, select, os
import math
import time
if os.name == 'nt':
  import msvcrt
else:
  import tty, termios

BURGER_MAX_LIN_VEL = 0.22
BURGER_MAX_ANG_VEL = 2.84

WAFFLE_MAX_LIN_VEL = 0.26
WAFFLE_MAX_ANG_VEL = 1.82

LIN_VEL_STEP_SIZE = 0.01
ANG_VEL_STEP_DEGR = 15.0
ANG_VEL_STEP_SIZE = 0.01
#ANG_VEL_STEP_DEGR*math.pi/180 #+-15 graden



msg = """
Control Your TurtleBot3!
---------------------------
Moving around:
        w
   a    s    d
        x

w/x : increase/decrease linear velocity (Burger : ~ 0.22, Waffle and Waffle Pi : ~ 0.26)
a/d : increase/decrease angular velocity (Burger : ~ 2.84, Waffle and Waffle Pi : ~ 1.82)

space key, s : force stop

CTRL-C to quit
"""

e = """
Communications Failed
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

def vels(target_linear_vel, target_angular_vel):
    return "currently:\tlinear vel %s\t angular vel %s " % (target_linear_vel,target_angular_vel)

def makeSimpleProfile(output, input, slop):
    if input > output:
        output = min( input, output + slop )
    elif input < output:
        output = max( input, output - slop )
    else:
        output = input

    return output

def constrain(input, low, high):
    if input < low:
      input = low
    elif input > high:
      input = high
    else:
      input = input

    return input

def checkLinearLimitVelocity(vel):
    if turtlebot3_model == "burger":
      vel = constrain(vel, -BURGER_MAX_LIN_VEL, BURGER_MAX_LIN_VEL)
    elif turtlebot3_model == "waffle" or turtlebot3_model == "waffle_pi":
      vel = constrain(vel, -WAFFLE_MAX_LIN_VEL, WAFFLE_MAX_LIN_VEL)
    else:
      vel = constrain(vel, -BURGER_MAX_LIN_VEL, BURGER_MAX_LIN_VEL)

    return vel

def checkAngularLimitVelocity(vel):
    if turtlebot3_model == "burger":
      vel = constrain(vel, -BURGER_MAX_ANG_VEL, BURGER_MAX_ANG_VEL)
    elif turtlebot3_model == "waffle" or turtlebot3_model == "waffle_pi":
      vel = constrain(vel, -WAFFLE_MAX_ANG_VEL, WAFFLE_MAX_ANG_VEL)
    else:
      vel = constrain(vel, -BURGER_MAX_ANG_VEL, BURGER_MAX_ANG_VEL)
    return vel

def turn(angle_degree,control_linear_vel, target_linear_vel,control_angular_vel, target_angular_vel):
    for i in range(27):#snelheid van 15 graden per seconde
        target_angular_vel = checkAngularLimitVelocity(target_angular_vel + ANG_VEL_STEP_SIZE)
        control_linear_vel, control_angular_vel = controleEnPublish(control_linear_vel, target_linear_vel,control_angular_vel, target_angular_vel)
    t = angle_degree/ANG_VEL_STEP_DEGR
    print("seconds:" + str(t))
    time.sleep(t+0.2)
    stop()
    return 

def goStraight(length, control_linear_vel, target_linear_vel,control_angular_vel, target_angular_vel):
    for i in range(10):
        target_linear_vel = checkLinearLimitVelocity(target_linear_vel + LIN_VEL_STEP_SIZE)
        control_linear_vel, control_angular_vel = controleEnPublish(control_linear_vel, target_linear_vel,control_angular_vel, target_angular_vel)
    t = length/target_linear_vel
    time.sleep(t)
    return 

def stop():
    target_linear_vel   = 0.0
    control_linear_vel  = 0.0
    target_angular_vel  = 0.0
    control_angular_vel = 0.0
    print(vels(target_linear_vel, target_angular_vel))
    control_linear_vel, control_angular_vel = controleEnPublish(control_linear_vel, target_linear_vel,control_angular_vel, target_angular_vel)
    return



def controleEnPublish(control_linear_vel, target_linear_vel,control_angular_vel, target_angular_vel):
    twist = Twist()

    control_linear_vel = makeSimpleProfile(control_linear_vel, target_linear_vel, (LIN_VEL_STEP_SIZE))
    twist.linear.x = control_linear_vel; twist.linear.y = 0.0; twist.linear.z = 0.0

    control_angular_vel = makeSimpleProfile(control_angular_vel, target_angular_vel, (ANG_VEL_STEP_SIZE))
    twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = control_angular_vel
    print(vels(target_linear_vel, target_angular_vel))
    pub.publish(twist)
    return control_linear_vel, control_angular_vel ;


def callback_fun(msg):
	print(msg.function,msg.distance,msg.angle)
	if msg.function == 'turn':
		turn(msg.angle,0,0,0,0)
	elif msg.function == 'straigth':
		goStraigth(msg.distance,0,0,0,0)
	elif msg.function == 'stop':
		stop()
	else:
		stop()

if __name__=="__main__":
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)


    rospy.init_node('wheel_control')
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    rospy.Subscriber("path", path_message, callback_fun)


    turtlebot3_model = rospy.get_param("model", "burger")

    status = 0
    target_linear_vel   = 0.0
    target_angular_vel  = 0.0
    control_linear_vel  = 0.0
    control_angular_vel = 0.0

    try:
        print(msg)
        while(1):
            key = getKey()
            if key == 'f': 
                goStraight(1,control_linear_vel, target_linear_vel,control_angular_vel, target_angular_vel)
                
            elif key == 't' :
                angle_degree = 90
                turn(angle_degree,control_linear_vel, target_linear_vel,control_angular_vel, target_angular_vel)
            elif key == 'r' :
                drawRectangle(control_linear_vel, target_linear_vel,control_angular_vel, target_angular_vel)  
                
#vaste angular snelheid <-> duurtijd => hoek
            elif key == ' ' or key == 's' :#dit in een functie steken stoppen -> hoe meerdere argumentern terug geven?
                target_linear_vel   = 0.0
                control_linear_vel  = 0.0
                target_angular_vel  = 0.0
                control_angular_vel = 0.0
                print(vels(target_linear_vel, target_angular_vel))
                controleEnPublish(control_linear_vel, target_linear_vel,control_angular_vel, target_angular_vel)
            else:
                if (key == '\x03'):
                    break

    except:
        print(e)

    finally:
        twist = Twist()
        twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
        twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0
        pub.publish(twist)

    if os.name != 'nt':
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
