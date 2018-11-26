#include "ros/ros.h"
#include "iiwaRos.h"
#include <cstdlib>

#include "geometry_msgs/WrenchStamped.h"
#include "std_msgs/Float64.h"


std_msgs::Float64 pt_wrench_x;
std_msgs::Float64 pt_wrench_y;
std_msgs::Float64 pt_wrench_z;
std_msgs::Float64 pt_bz;

void wrench_callback(const geometry_msgs::WrenchStamped& msg1)
{
  pt_wrench_x.data=msg1.wrench.force.x;
  pt_wrench_y.data=msg1.wrench.force.y;
  pt_wrench_z.data=msg1.wrench.force.z;
  pt_bz.data=10;
  
//ROS_INFO("listener_wrench:%f,%f,%f", msg1.wrench.force.x,msg1.wrench.force.y,msg1.wrench.force.z);
}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "pt_record_F");

  ros::NodeHandle n;
  ros::Subscriber wrench_sub = n.subscribe("/iiwa/state/CartesianWrench", 1000, wrench_callback);
  ros::Publisher wrench_pubx = n.advertise<std_msgs::Float64>("force_x", 1000);
  ros::Publisher wrench_puby = n.advertise<std_msgs::Float64>("force_y", 1000);
  ros::Publisher wrench_pubz = n.advertise<std_msgs::Float64>("force_z", 1000);
  ros::Publisher BZ = n.advertise<std_msgs::Float64>("BZ", 1000);
  ros::Rate loop_rate(256);
  while (ros::ok()){

  wrench_pubx.publish(pt_wrench_x);
  wrench_puby.publish(pt_wrench_y);
  wrench_pubz.publish(pt_wrench_z);
  BZ.publish(pt_bz);

  ros::spinOnce();
  loop_rate.sleep();
 }
 return 0;
  
}
