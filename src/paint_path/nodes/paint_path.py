#!/usr/bin/env python

from gazebo_msgs.srv import GetModelState
import rospy
	

if __name__ == '__main__':
	try:
		model_coordinates = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
		model_name = "turtlebot3_burger"
		relative_entity_name = "base_footprint"

		resp_coordinates = model_coordinates(model_name, relative_entity_name)
		print("Model: " + str(model_name))
		print("X-coordinate : " + str(resp_coordinates.pose.position.x))
		print("Y-coordinate: " + str(resp_coordinates.pose.position.y))

	except rospy.ServiceException as e:
		rospy.loginfo("Get Model State service call failed:  {0}".format(e))
	rospy.spin()
