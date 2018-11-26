#include "ros/ros.h"
#include "iiwaRos.h"
#include <cstdlib>

#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/Int16.h"


std_msgs::Int16 pt_position_x;
std_msgs::Int16 pt_position_y;
std_msgs::Int16 pt_position_z;
std_msgs::Int16 pt_bz;

void pose_callback(const geometry_msgs::PoseStamped& msg1)
{
  pt_position_x.data=msg1.pose.position.x;
  pt_position_y.data=msg1.pose.position.y;
  pt_position_z.data=msg1.pose.position.z;
  
//ROS_INFO("listener_wrench:%f,%f,%f", msg1.wrench.force.x,msg1.wrench.force.y,msg1.wrench.force.z);
}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "pt_record_X");

  ros::NodeHandle n;
  ros::Subscriber pose_sub = n.subscribe("/iiwa/state/CartesianPose", 1000, pose_callback);
  ros::Publisher position_pubx = n.advertise<std_msgs::Int16>("position_x", 1000);
  ros::Publisher position_puby = n.advertise<std_msgs::Int16>("position_y", 1000);
  ros::Publisher position_pubz = n.advertise<std_msgs::Int16>("position_z", 1000);
  ros::Rate loop_rate(50);
  while (ros::ok()){

  position_pubx.publish(pt_position_x);
  position_puby.publish(pt_position_y);
  position_pubz.publish(pt_position_z);

  ros::spinOnce();
  loop_rate.sleep();
 }
 return 0;
  
}
