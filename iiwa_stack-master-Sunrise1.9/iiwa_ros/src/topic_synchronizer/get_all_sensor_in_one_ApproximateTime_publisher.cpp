#include "ros/ros.h"
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

using namespace message_filters;
//tf2_ros::TransformBroadcaster tf2_broadcaster;
geometry_msgs::TransformStamped transformStamped;

ros::Publisher sensor_pub;
// ros::Publisher sensor_pub1;
// ros::Publisher sensor_pub2;

void callback(const geometry_msgs::WrenchStampedConstPtr& netft_data_msg, const geometry_msgs::PoseArrayConstPtr& targets);  

int main(int argc, char** argv)
{
  ros::init(argc, argv, "get_all_sensor_in_one");

  ros::NodeHandle nh;

  sensor_pub = nh.advertise<geometry_msgs::TransformStamped>("/sensor_together", 1000);
  // sensor_pub1 = nh.advertise<geometry_msgs::WrenchStamped>("/sensor_ati", 1000);
  // sensor_pub2 = nh.advertise<geometry_msgs::PoseArray>("/sensor_ndi", 1000);

  message_filters::Subscriber<geometry_msgs::WrenchStamped> force_sub(nh, "/netft_data", 1);    
  message_filters::Subscriber<geometry_msgs::PoseArray> pose_sub(nh, "/polaris_sensor/targets", 1);    
  //message_filters::TimeSynchronizer<geometry_msgs::WrenchStamped, geometry_msgs::PoseArray> sync(force_sub, pose_sub, 2);       
  typedef sync_policies::ApproximateTime<geometry_msgs::WrenchStamped, geometry_msgs::PoseArray> MySyncPolicy;
  // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
  Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), force_sub, pose_sub);
  sync.registerCallback(boost::bind(&callback, _1, _2));                   

  // sensor_pub.publish(sync);

  ros::spin();

  return 0;
}

void callback(const geometry_msgs::WrenchStampedConstPtr& netft_data_msg, const geometry_msgs::PoseArrayConstPtr& targets)  
{

  // static tf2_ros::TransformBroadcaster tf2_broadcaster;
  // geometry_msgs::TransformStamped transformStamped;

  transformStamped.header.stamp = ros::Time::now();
  transformStamped.header.frame_id = "ts_frame";
  transformStamped.child_frame_id = "prism_frame";
  transformStamped.transform.translation.x = targets->poses[0].position.x;
  transformStamped.transform.translation.y = targets->poses[0].position.y;
  transformStamped.transform.translation.z = targets->poses[0].position.z;
  tf2::Quaternion q;
  transformStamped.transform.rotation.x = netft_data_msg->wrench.force.x;
  transformStamped.transform.rotation.y = netft_data_msg->wrench.force.y;
  transformStamped.transform.rotation.z = netft_data_msg->wrench.force.z;
  transformStamped.transform.rotation.w = targets->poses[0].orientation.w;

  //tf2_broadcaster.sendTransform(transformStamped);

  ROS_INFO("Synchronization successful [%f]", transformStamped.transform.rotation.z);//transformStamped.transform.rotation.z

  sensor_pub.publish(transformStamped);
  // sensor_pub1.publish(netft_data_msg);
  // sensor_pub2.publish(targets);
}