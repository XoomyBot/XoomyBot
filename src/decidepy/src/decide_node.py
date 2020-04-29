#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from fiducial_msgs.msg import *
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import numpy as np
import cv2


def callback(data):
	global fids,draw_ids,ids
	fids=data.fiducials
	if(len(fids)>0):
		rospy.loginfo(rospy.get_caller_id() + "nr of id %d",len(fids))
	for i in range (len(fids)):
		fid=fids[i];
    		rospy.loginfo(rospy.get_caller_id() + "I saw %d", fid.fiducial_id)
		ids.append(fid.fiducial_id)

def img_callback(data):
    try:
      cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
    for i in range (len(fids)):
	fid=fids[i];
	pts=np.array([[fid.x0,fid.y0],[fid.x1,fid.y1],[fid.x2,fid.y2],[fid.x3,fid.y3]])
	cv_image=cv2.polylines(cv_image, np.int32([pts]),  1, (255,0,255))

		
    if (0 in ids):
	pnt0=[fid.x2,fid.y2]
	cv_image = cv2.circle(cv_image, pnt0, 1, (0,0,255), 20) 
	if (1 in ids):
		pnt1=[fid.x3,fid.y3]
		cv_image = cv2.circle(cv_image, pnt1, 1, (0,0,255), 20) 
		if (2 in ids):
			pnt2=[fid.x1,fid.y1]
			cv_image = cv2.circle(cv_image, pnt2, 1, (0,0,255), 20) 
			if (3 in ids):
					pnt3=[fid.x0,fid.y0]
					pts=np.array([pnt0,pnt1,pnt2,pnt3])
					cv_image = cv2.circle(cv_image, pnt3, 1, (0,0,255), 20) 
					cv_image=cv2.polylines(cv_image, np.int32([pts]),  1, (0,0,255))
	
    cv2.imshow("Image window", cv_image)
    cv2.waitKey(3)

    try:
      image_pub.publish(bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    except CvBridgeError as e:
      print(e)

def decide():
	global fids,draw_ids,ids
	ids=[]
	fids=[]
	draw_ids=[0,1,2,3]
	rospy.init_node('decide2', anonymous=True)
    	rospy.Subscriber("/fiducial_vertices", FiducialArray, callback)
    	rospy.Subscriber("/camera", Image,img_callback)
	global image_pub
	image_pub = rospy.Publisher("image_topic_2",Image)
	global bridge
	bridge = CvBridge()

    # spin() simply keeps python from exiting until this node is stopped
    	rospy.spin()

if __name__ == '__main__':
    decide()
