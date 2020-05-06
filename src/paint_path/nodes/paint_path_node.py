#!/usr/bin/env python

from gazebo_msgs.srv import GetModelState
import rospy
import sys, select, os
if os.name == 'nt':
  import msvcrt
else:
  import tty, termios


def show_gazebo_coor():

	model_coordinates = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
	model_name = 'turtlebot3_burger'
	relative_entity_name = 'wheel_left_link'
	resp_coordinates = model_coordinates(model_name, relative_entity_name)
	print("Model: " + str(model_name))
	print("X-coordinate : " + str(resp_coordinates.pose.position.x))
	print("Y-coordinate: " + str(resp_coordinates.pose.position.y))
	return

	
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

if __name__ == '__main__':
	if os.name != 'nt':
		settings = termios.tcgetattr(sys.stdin)
	rospy.init_node('paint_path_node')

	try:
		while(1):
			key = getKey()
			
			model_coordinates = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
			model_name = 'turtlebot3_burger'
			relative_entity_name = 'base_footprint'

			resp_coordinates = model_coordinates(model_name, relative_entity_name)
			print("Model: " + str(model_name))
			print("X-coordinate : " + str(resp_coordinates.pose.position.x))
			print("Y-coordinate: " + str(resp_coordinates.pose.position.y))
		
			
			if (key == '\x03'):
				break
	except rospy.ServiceException as e:
		rospy.loginfo("Get Model State service call failed:  {0}".format(e))
			
		
