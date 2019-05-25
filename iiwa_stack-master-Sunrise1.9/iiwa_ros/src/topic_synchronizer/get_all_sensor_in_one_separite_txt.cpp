#include "ros/ros.h"
#include "iiwaRos.h"
#include <sstream>
#include <iostream>
#include <fstream>
#include <string>
#include <stdio.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/WrenchStamped.h>       
#include <geometry_msgs/PoseArray.h>  
#include "std_msgs/Float64.h"
#include <iiwa_msgs/ForceAndPosition.h>

using namespace std;
using namespace message_filters;

void callback_ati(const geometry_msgs::WrenchStamped& msg1);
void callback_ndi(const geometry_msgs::PoseArray& msg2);

int main(int argc, char** argv)
{
  ros::init(argc, argv, "get_all_sensor_in_one");

  ros::NodeHandle nh;

  ros::Subscriber force_sub1 = nh.subscribe("/netft_data", 1000, callback_ati);
  ros::Subscriber pose_sub1 = nh.subscribe("/polaris_sensor/targets", 1000, callback_ndi);

  ros::spin();

  return 0;
}

void callback_ati(const geometry_msgs::WrenchStamped& msg1)
{
  ofstream outfile;
  //name of txt: impedance_estimation_test_5_0515.txt
  outfile.open("1ati_separite_time_0524_4.txt", ios::binary | ios::app | ios::in | ios::out);
  outfile<<msg1.header.stamp<<"  "
  <<msg1.wrench.force.x<<"  "
  <<msg1.wrench.force.y<<"  "
  <<msg1.wrench.force.z<<"  "
  <<msg1.wrench.torque.x<<"  "
  <<msg1.wrench.torque.y<<"  "
  <<msg1.wrench.torque.z<<"\n";
  outfile.close();
}

void callback_ndi(const geometry_msgs::PoseArray& msg2)
{
  ofstream outfile;
  //name of txt: impedance_estimation_test_5_0515.txt
  outfile.open("1ndi_separite_time_0524_4.txt", ios::binary | ios::app | ios::in | ios::out);
  outfile<<msg2.header.stamp<<"  "
  <<msg2.poses[0].position.x<<"  "
  <<msg2.poses[0].position.y<<"  "
  <<msg2.poses[0].position.z<<"  "
  <<msg2.poses[0].orientation.x<<"  "
  <<msg2.poses[0].orientation.y<<"  "
  <<msg2.poses[0].orientation.z<<"  "
  <<msg2.poses[0].orientation.w<<"\n";
  outfile.close();
  ROS_INFO("Synchronization successful [%f]", msg2.poses[0].position.x);
}