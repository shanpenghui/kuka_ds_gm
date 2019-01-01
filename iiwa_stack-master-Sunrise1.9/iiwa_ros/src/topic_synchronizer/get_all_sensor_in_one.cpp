// #include <ros/ros.h>
// #include <sensor_msgs/Image.h>
// #include <message_filters/subscriber.h>
// #include <message_filters/synchronizer.h>
// #include <message_filters/sync_policies/approximate_time.h>
// #include <geometry_msgs/Wrench.h>       
// #include <geometry_msgs/PoseArray.h>  

// // using namespace message_filters;

// class Node
// {
//  public:
//   Node()
//   {
//     sub_1_.subscribe(nh_, "/netft_data", 1);
//     sub_2_.subscribe(nh_, "/targets", 1);
//     sync_.reset(new Sync(MySyncPolicy(10), sub_1_, sub_2_));
//     sync_->registerCallback(boost::bind(&Node::callback, this, _1, _2));
//   }

//   void callback(const geometry_msgs::WrenchConstPtr &netft_data, const geometry_msgs::PoseArrayConstPtr &targets)
//   {
//     ROS_INFO("Synchronization successful");
//   }

//  private:
//   ros::NodeHandle nh_;
//   message_filters::Subscriber<geometry_msgs::Wrench> sub_1_;
//   message_filters::Subscriber<geometry_msgs::PoseArray> sub_2_;

//   typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::Wrench, geometry_msgs::PoseArray> MySyncPolicy;
//   typedef message_filters::Synchronizer<MySyncPolicy> Sync;
//   boost::shared_ptr<Sync> sync_;
// };

// int main(int argc, char **argv)
// {
//   ros::init(argc, argv, "synchronizer");

//   Node synchronizer;

//   ros::spin();
// }


//////////////////////////////////////第二个版本

#include "ros/ros.h"
#include <sstream>
#include <iostream>
#include <fstream>
#include <string>
#include <stdio.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/WrenchStamped.h>       
#include <geometry_msgs/PoseArray.h>  
#include "std_msgs/Float64.h"

void callback(const geometry_msgs::WrenchStampedConstPtr& netft_data, const geometry_msgs::PoseArrayConstPtr& targets)  //回调中包含多个消息
{

  static tf2_ros::TransformBroadcaster tf2_broadcaster;
  geometry_msgs::TransformStamped transformStamped;

  transformStamped.header.stamp = ros::Time::now();
  transformStamped.header.frame_id = "ts_frame";
  transformStamped.child_frame_id = "prism_frame";
  transformStamped.transform.translation.x = targets->poses[0].position.x;
  transformStamped.transform.translation.y = targets->poses[0].position.y;
  transformStamped.transform.translation.z = targets->poses[0].position.z;
  tf2::Quaternion q;
  transformStamped.transform.rotation.x = netft_data->wrench.force.x;
  transformStamped.transform.rotation.y = netft_data->wrench.force.y;
  transformStamped.transform.rotation.z = netft_data->wrench.force.z;
  transformStamped.transform.rotation.w = targets->poses[0].orientation.w;

  tf2_broadcaster.sendTransform(transformStamped);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "get_all_sensor_in_one");

  ros::NodeHandle nh;

  message_filters::Subscriber<geometry_msgs::WrenchStamped> force_sub(nh, "/netft_data", 1);    // ati 输入
  message_filters::Subscriber<geometry_msgs::PoseArray> pose_sub(nh, "/targets", 1);     // ndi 输入
  message_filters::TimeSynchronizer<geometry_msgs::WrenchStamped, geometry_msgs::PoseArray> sync(force_sub, pose_sub, 10);       // 同步
  sync.registerCallback(boost::bind(&callback, _1, _2));                   // 回调

  //回调后记录下来；


  ros::spin();

  return 0;
}



///////、、、、、、、、、、、、、、、、、、、、之前除了同步都好使的版本

// #include "ros/ros.h"
// #include <sstream>
// #include <iostream>
// #include <fstream>
// #include <string>
// #include <stdio.h>
// #include <message_filters/subscriber.h>
// #include <message_filters/time_synchronizer.h>
// #include <geometry_msgs/Wrench.h>       
// #include <geometry_msgs/PoseArray.h>  
// #include "std_msgs/Float64.h"

// using namespace std;
// using namespace geometry_msgs;
// using namespace message_filters;

// std_msgs::Float64 fx;std_msgs::Float64 fy;std_msgs::Float64 fz;std_msgs::Float64 wx;std_msgs::Float64 wy;
// std_msgs::Float64 wz;std_msgs::Float64 px;std_msgs::Float64 py;std_msgs::Float64 pz;std_msgs::Float64 ox;std_msgs::Float64 oy;
// std_msgs::Float64 oz;std_msgs::Float64 ow;


// void callback(const geometry_msgs::Wrench& netft_data, const geometry_msgs::PoseArray& targets)  //回调中包含多个消息
// {

//   fx.data=netft_data.force.x;
//   fy.data=netft_data.force.y;
//   fz.data=netft_data.force.z;
//   wx.data=netft_data.torque.x;
//   wy.data=netft_data.torque.y;
//   wz.data=netft_data.torque.z;

//   px.data=targets.poses[0].position.x;//[{position: [1,2,3]}, {position: [4,5,6]}]
//   py.data=targets.poses[0].position.y;
//   pz.data=targets.poses[0].position.z;
//   ox.data=targets.poses[0].orientation.x;
//   oy.data=targets.poses[0].orientation.y;
//   oz.data=targets.poses[0].orientation.z;
//   ow.data=targets.poses[0].orientation.w;
// }

// int main(int argc, char** argv)
// {
//   ros::init(argc, argv, "get_all_sensor_in_one");

//   ros::NodeHandle nh;

//   message_filters::Subscriber<geometry_msgs::Wrench> force_sub(nh, "netft_data", 1);    // ati 输入
//   message_filters::Subscriber<geometry_msgs::PoseArray> pose_sub(nh, "targets", 1);     // ndi 输入
//   TimeSynchronizer<geometry_msgs::Wrench, geometry_msgs::PoseArray> sync(force_sub, pose_sub, 10);       // 同步
//   // sync.registerCallback(boost::bind(&callback, _1, _2));                   // 回调

//   //回调后记录下来；


//   ros::spin();

//   return 0;
// }