#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from fiducial_msgs.msg import *
from wheel_control.msg import path_message
import sys, select, os
import math
import time
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import numpy as np
import cv2
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

def callback(data):
	global fids,ids,pnt0,pnt1,pnt3,pnt2,last
	fids=data.fiducials
	ids=[]
	for i in range (len(fids)):
		fid=fids[i];
		ids.append(fid.fiducial_id)
		if (fid.fiducial_id!=last):
			last=fid.fiducial_id
			if (fid.fiducial_id==4):
				print("teken rechthoek")
				drawRectangle(1,2,90)
			elif (fid.fiducial_id==3):
				print("teken driehoek")
				drawTriangle(2,120)
			elif (fid.fiducial_id==6):
				print("stop")
				images=0
				message = path_message()
				message.function = "stop"
				message.distance= 0.0
				message.angle = 0.0
				pub.publish(message)
			elif (fid.fiducial_id==8):
				pnt0=(int(fid.x2),int(fid.y2))
				
			elif (fid.fiducial_id==9):
				pnt1=(int(fid.x3),int(fid.y3))
				
			elif (fid.fiducial_id==10):
				pnt2=(int(fid.x1),int(fid.y1))
				
			elif (fid.fiducial_id==11):
				pnt3=(int(fid.x0),int(fid.y0))
		 

def img_callback(data):
    global fids,ids,images,opened
    try:
      cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
    mask = np.zeros(cv_image.shape, cv_image.dtype)
    res = np.zeros(cv_image.shape, cv_image.dtype)
    for i in range (len(fids)):
	fid=fids[i];
	pts=np.array([[fid.x0,fid.y0],[fid.x1,fid.y1],[fid.x2,fid.y2],[fid.x3,fid.y3]])
	cv_image=cv2.polylines(cv_image, np.int32([pts]),  1, (255,0,255))
	    
    if(all(x in ids for x in [8,9,10,11])):
	images+=1;
	rect=[pnt0,pnt1,pnt3,pnt2]
	for pt in rect:
		cv_image = cv2.circle(cv_image, pt, 1, (0,255,0), 5) 
	cv_image=cv2.polylines(cv_image, np.int32([rect]),  1, (0,0,255))

	if (images==40):
		mask =cv2.fillPoly(mask, np.int32([rect]),(255,255,255),1)
		gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
		mask = cv2.cvtColor(mask, cv2.COLOR_BGR2GRAY)
		res = cv2.bitwise_and(gray,gray,mask = mask)
		kernel = np.ones((5,5),np.uint8)
		ret, th = cv2.threshold(res, 150, 255, 0)
		opened = cv2.morphologyEx(th, cv2.MORPH_OPEN, kernel)
		#closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, kernel)
		move=Moves()
		print("draw custom")
		print(move)
		drawCustom(move)
		   
    cv2.waitKey(3)

    try:
      image_pub.publish(bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    except CvBridgeError as e:
      print(e)
def Moves():
	image=opened
	empty = np.zeros(image.shape, image.dtype)
	hoek=()
	vooruit=()
	_,contours, _ = cv2.findContours(image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
	points = []
	print("Number of Contours found = " + str(len(contours))) 
	cnt=sorted(contours, key=cv2.contourArea, reverse=True)
	approx = cv2.approxPolyDP(cnt[1], 0.01*cv2.arcLength(cnt[1], True), True)
      	points.append(approx)
      	cv2.drawContours(empty, [approx], -1, (255,255,255), 1) 
  	cont = []
	cv2.imshow("contours",empty)
	for u in points: 
		shape = []
    		x = 0
    		for v in u:
      			for w in v:
        			if x == 0:
						#het onthouden van de start om het contour te sluiten(anders zou er 1lijn vergeten worden)
          				eerste_x = w[0] 
          				eerste_y = w[1]
          				eerste = w
        			shape.append(w)
      			x = 1
    		shape.append(eerste)
    		cont.append(shape)
  	scaler = int((pnt0[0]**2 + pnt1[0]**2)**0.5)
	scaler= scaler/2
  	x0 = 0 #oorsprong
 	y0 = 0
  	theta0 = 0 #rechts
  	moves = []
  	count = 0
  	for a in cont:
    		for b in a:
      			x1 = b[0]
      			y1 = b[1]
      			Dy = y1-y0
      			Dx = x1-x0
      			theta = math.atan(Dy/Dx)*(360/(2*math.pi))#omvormen naar graden
      			if count < 2:
        			dtheta = theta
      			else:
        			dtheta = theta-theta0 #foutje met nan!!
      			if Dx < 0:
        			if dtheta > 0:
          				dtheta = dtheta-180 #graden!!
          				theta = theta-180
        			else:
          				dtheta = dtheta+180 #graden!!
          				theta = theta+180
	      		distance = ((Dx**2 + Dy**2)**0.5)/scaler
			hoek = ("turn", 0, dtheta)
			vooruit =("straight", distance, 0)
			moves.append(hoek)
			moves.append(vooruit)
			x0 = x1
			y0 = y1
			theta0 = theta
			dtheta0 = dtheta
			count = count+1
	return moves
def decide():
	global fids,ids,images,mask
	images=0
	ids=[]
	fids=[]
    	rospy.Subscriber("/camera", Image,img_callback)
	global image_pub
	image_pub = rospy.Publisher("cv_image",Image,queue_size=100)
	global bridge
	bridge = CvBridge()

  

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
	message = path_message()
       	message.function = func
	message.distance = dist
	message.angle = ang
        pub.publish(message)
def drawCustom(data):
    print("pulbiceren")
    for i in range(len(data)):
	publish_message(data[i][0],data[i][1],data[i][2])
    	publish_message("stop",0,0)
    return   
	
	
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
	global last
	last=-1

	if os.name != 'nt':
		settings = termios.tcgetattr(sys.stdin)

	rospy.init_node('decide_path_node')
	pub = rospy.Publisher('path', path_message, queue_size=10)
    	rospy.Subscriber("/fiducial_vertices", FiducialArray, callback, queue_size=100)
    	decide()

	try:
		
		print(msg)
		while(1):

			key = getKey()
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
