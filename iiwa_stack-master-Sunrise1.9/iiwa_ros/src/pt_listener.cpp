#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"



void chatterCallback(const geometry_msgs::PoseStamped& msg)
{
  ROS_INFO("%f,%f,%f", msg.pose.position.x,msg.pose.position.y,msg.pose.position.z);
  ROS_INFO("%f,%f,%f,%f", msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w);
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "pt_listener");

  
  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("/iiwa/state/CartesianPose", 1000, chatterCallback);

  ros::spin();

  return 0;
}
