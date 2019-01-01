#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <iostream>
#include "iiwaRos.h"
#include "geometry_msgs/PoseStamped.h"


geometry_msgs::PoseStamped pose_pt1;
void pose_chatterCallback(const geometry_msgs::PoseStamped& msg1)
{
  pose_pt1.pose.position.x=msg1.pose.position.x;
  pose_pt1.pose.position.y=msg1.pose.position.y;
  pose_pt1.pose.position.z=msg1.pose.position.z;
  pose_pt1.pose.orientation.x=msg1.pose.orientation.x;
  pose_pt1.pose.orientation.y=msg1.pose.orientation.y;
  pose_pt1.pose.orientation.z=msg1.pose.orientation.z;
  pose_pt1.pose.orientation.w=msg1.pose.orientation.w;

  ROS_INFO("%f,%f,%f", pose_pt1.pose.position.x,pose_pt1.pose.position.y,pose_pt1.pose.position.z);
}
int main(int argc, char **argv)
{
 
  ros::init(argc, argv, "pt_go");
  ros::NodeHandle n;
  ros::Subscriber pose_sub = n.subscribe("POSE_pt", 1000, pose_chatterCallback);
  ros::Publisher chatter_pub = n.advertise<geometry_msgs::PoseStamped>("/iiwa/command/CartesianPose", 1000);

  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    geometry_msgs::PoseStamped msg;

    msg.pose.position.x=pose_pt1.pose.position.x;
    msg.pose.position.y=pose_pt1.pose.position.y;
    msg.pose.position.z=pose_pt1.pose.position.z;

    msg.pose.orientation.x=pose_pt1.pose.orientation.x;
    msg.pose.orientation.y=pose_pt1.pose.orientation.y;
    msg.pose.orientation.z=pose_pt1.pose.orientation.z;
    msg.pose.orientation.w=pose_pt1.pose.orientation.w;
   //ROS_INFO("%f,%f,%f", msg.pose.position.x,msg.pose.position.y,msg.pose.position.z);

    chatter_pub.publish(msg);
    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}
/*
  position: 
    x: 0.421120645169
    y: 0.0205836859991
    z: 0.581226265672
  orientation: 
    x: -0.353685581886
    y: 0.935302317142
    z: -0.00418645251225
    w: 0.00992877288998*/
