/**
**  Simple ROS Node
**/
#include <ros/ros.h>
#include "fiducial_msgs/Fiducial.h"
#include "fiducial_msgs/FiducialArray.h"
#include "fiducial_msgs/FiducialTransform.h"
#include "fiducial_msgs/FiducialTransformArray.h"

void chatterCallback(const fiducial_msgs::FiducialArray::ConstPtr &msg)
{
for (size_t i = 0; i < msg->fiducials.size(); i++) {
	const fiducial_msgs::Fiducial &f = msg->fiducials[i];
	ROS_INFO("I saw: [%d] %d", f.fiducial_id,f.direction);
	}

}
int main(int argc, char* argv[])
{
  // This must be called before anything else ROS-related
  ros::init(argc, argv, "decide_node");
  ros::Subscriber verticesSub;
	
  // Create a ROS node handle
  ros::NodeHandle nh;

  ROS_INFO("Decide node ready!");
  ros::Subscriber sub = nh.subscribe("/fiducial_vertices", 1000, chatterCallback);


  // Don't exit the program.
  ros::spin();
}
