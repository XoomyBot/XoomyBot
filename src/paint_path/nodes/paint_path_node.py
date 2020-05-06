#!/usr/bin/env python

from gazebo_msgs.srv import GetModelState
import rospy
import sys, select, os
if os.name == 'nt':
  import msvcrt
else:
  import tty, termios
import cv2
from PIL import Image, ImageDraw, ImageFont
import matplotlib.pyplot as plt 
import numpy as np
	
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

def draw(x,y,color, image,img_cv):
	img = Image.open(image)
	pos = tuple([x,y])
	w,h=500,500 
	#newsize = (w,h) 
	k = int(w/2) 
	##imgres = img.resize(newsize) 
	#draw the point on the path.png file75
	draw = ImageDraw.Draw(img)
	i = 0
	j = 0

	for i in range(-3,i<3,1):
		for j in range(-3,j<3,1):
			draw.point([(k+pos[0]+i,k+pos[1]+j)],fill=color)
	img_cv = cv2.circle(img_cv, (int(k+x),int(k+y)), 5, (255,255,255), -1) 
	cv2.imshow("teken",img_cv)
	cv2.waitKey(1)
	img.save(image)	
		

if __name__ == '__main__':


	if os.name != 'nt':
		settings = termios.tcgetattr(sys.stdin)
	rospy.init_node('paint_path_node')

	try:
		imagePath = "path.png"
		backColor = 'white' 
        	img_cv = np.zeros((500,500))
		w,h=500,500 
		k = int(w/2) 
		img = Image.new('RGB', (w, h), color = backColor) 
		img.save(imagePath)

		while(1):
			key = getKey()
			
			model_coordinates = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
			model_name = 'turtlebot3_burger'
			#leave relative empty for model itself
			
			resp_coordinates = model_coordinates(model_name, '')

			#print(resp_coordinates)
			print('Status.success = ', resp_coordinates.success)
			print("Model: " + str(model_name))
			print("X-coordinate : " + str(resp_coordinates.pose.position.x))
			print("Y-coordinate: " + str(resp_coordinates.pose.position.y))
			draw(100*resp_coordinates.pose.position.x,100*resp_coordinates.pose.position.y, "red", imagePath,img_cv)
			
			if (key == '\x03'):
				break
	except rospy.ServiceException as e:
		rospy.loginfo("Get Model State service call failed:  {0}".format(e))
			
		
