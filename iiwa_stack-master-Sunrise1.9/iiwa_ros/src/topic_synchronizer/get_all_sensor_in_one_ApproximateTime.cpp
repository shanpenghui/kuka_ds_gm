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

// geometry_msgs::TransformStamped transformStamped;//didn't have torque and rotation
iiwa_msgs::ForceAndPosition ForceAndPositionInExperiment;

//ros::Publisher sensor_pub;
// ros::Publisher sensor_pub1;
// ros::Publisher sensor_pub2;


void callback(const geometry_msgs::WrenchStampedConstPtr& netft_data_msg, const geometry_msgs::PoseArrayConstPtr& targets);  //回调中包含多个消息

int main(int argc, char** argv)
{
  ros::init(argc, argv, "get_all_sensor_in_one");

  ros::NodeHandle nh;

  //ros::Publisher sensor_pub = nh.advertise<geometry_msgs::TransformStamped>("/sensor_together", 1000);
  // ros::Publisher sensor_pub1 = nh.advertise<geometry_msgs::WrenchStamped>("/sensor_ati", 1000);
  // ros::Publisher sensor_pub2 = nh.advertise<geometry_msgs::PoseArray>("/sensor_ndi", 1000);

  message_filters::Subscriber<geometry_msgs::WrenchStamped> force_sub(nh, "/netft_data", 1);    // ati 输入
  message_filters::Subscriber<geometry_msgs::PoseArray> pose_sub(nh, "/polaris_sensor/targets", 1);     // ndi 输入
  //message_filters::TimeSynchronizer<geometry_msgs::WrenchStamped, geometry_msgs::PoseArray> sync(force_sub, pose_sub, 2);       // 同步
  typedef sync_policies::ApproximateTime<geometry_msgs::WrenchStamped, geometry_msgs::PoseArray> MySyncPolicy;
  // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
  Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), force_sub, pose_sub);
  sync.registerCallback(boost::bind(&callback, _1, _2));                   // 回调

  //回调后记录下来；
  // sensor_pub.publish(sync);

  ros::spin();

  return 0;
}

void callback(const geometry_msgs::WrenchStampedConstPtr& netft_data_msg, const geometry_msgs::PoseArrayConstPtr& targets)  //回调中包含多个消息
{
//these part is using 3 position and 3 force, while not have rotation and torque
  // transformStamped.header.stamp = ros::Time::now();
  // transformStamped.header.frame_id = "ts_frame";
  // transformStamped.child_frame_id = "prism_frame";
  // transformStamped.transform.translation.x = targets->poses[0].position.x;
  // transformStamped.transform.translation.y = targets->poses[0].position.y;
  // transformStamped.transform.translation.z = targets->poses[0].position.z;
  // tf2::Quaternion q;
  // transformStamped.transform.rotation.x = netft_data_msg->wrench.force.x;
  // transformStamped.transform.rotation.y = netft_data_msg->wrench.force.y;
  // transformStamped.transform.rotation.z = netft_data_msg->wrench.force.z;
  // transformStamped.transform.rotation.w = targets->poses[0].orientation.w;

  ForceAndPositionInExperiment.wr_header.stamp = ros::Time::now();
  ForceAndPositionInExperiment.wr_header.frame_id = "ts_frame";
  ForceAndPositionInExperiment.wr_child_frame_id = "prism_frame";
  ForceAndPositionInExperiment.wr_px = targets->poses[0].position.x;
  ForceAndPositionInExperiment.wr_py = targets->poses[0].position.y;
  ForceAndPositionInExperiment.wr_pz = targets->poses[0].position.z;
  ForceAndPositionInExperiment.wr_pa = targets->poses[0].orientation.x;
  ForceAndPositionInExperiment.wr_pb = targets->poses[0].orientation.y;
  ForceAndPositionInExperiment.wr_pc = targets->poses[0].orientation.z;
  ForceAndPositionInExperiment.wr_pw = targets->poses[0].orientation.w;
  ForceAndPositionInExperiment.wr_fx = netft_data_msg->wrench.force.x;
  ForceAndPositionInExperiment.wr_fy = netft_data_msg->wrench.force.y;
  ForceAndPositionInExperiment.wr_fz = netft_data_msg->wrench.force.z;
  ForceAndPositionInExperiment.wr_fa = netft_data_msg->wrench.torque.x;
  ForceAndPositionInExperiment.wr_fb = netft_data_msg->wrench.torque.y;
  ForceAndPositionInExperiment.wr_fc = netft_data_msg->wrench.torque.z;

  //tf2_broadcaster.sendTransform(transformStamped);

  ROS_INFO("Synchronization successful [%f]", ForceAndPositionInExperiment.wr_px);//transformStamped.transform.rotation.z

  //sensor_pub.publish(transformStamped);
  // sensor_pub1.publish(netft_data_msg);
  // sensor_pub2.publish(targets);

  ofstream outfile;
  //name of txt: impedance_estimation_test_5_0515.txt
  outfile.open("impedance_estimation_test_4_0524.txt", ios::binary | ios::app | ios::in | ios::out);
  outfile<<ForceAndPositionInExperiment.wr_header.stamp<<"  "
  <<ForceAndPositionInExperiment.wr_px<<"  "
  <<ForceAndPositionInExperiment.wr_py<<"  "
  <<ForceAndPositionInExperiment.wr_pz<<"  "
  <<ForceAndPositionInExperiment.wr_pa<<"  "
  <<ForceAndPositionInExperiment.wr_pb<<"  "
  <<ForceAndPositionInExperiment.wr_pc<<"  "
  <<ForceAndPositionInExperiment.wr_pw<<"  "
  <<ForceAndPositionInExperiment.wr_fx<<"  "
  <<ForceAndPositionInExperiment.wr_fy<<"  "
  <<ForceAndPositionInExperiment.wr_fz<<"  "
  <<ForceAndPositionInExperiment.wr_fa<<"  "
  <<ForceAndPositionInExperiment.wr_fb<<"  "
  <<ForceAndPositionInExperiment.wr_fc<<"\n";
  outfile.close();
}

