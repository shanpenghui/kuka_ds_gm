#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include "iiwaRos.h"
#include "geometry_msgs/PoseStamped.h"

int main(int argc, char **argv)
{
 
  ros::init(argc, argv, "pt_experiment_start");
  ros::NodeHandle n;

  ros::Publisher chatter_pub = n.advertise<geometry_msgs::PoseStamped>("/iiwa/command/CartesianPose", 1000);

  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    geometry_msgs::PoseStamped msg;

    msg.pose.position.x=0.72;
    msg.pose.position.y=0.02;
    msg.pose.position.z=0.4;

    msg.pose.orientation.x=-0.353685581886;
    msg.pose.orientation.y=0.935302317142;
    msg.pose.orientation.z=-0.00418645251225;
    msg.pose.orientation.w=0.0099287728899;
   ROS_INFO("%f,%f,%f", msg.pose.position.x,msg.pose.position.y,msg.pose.position.z);

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
