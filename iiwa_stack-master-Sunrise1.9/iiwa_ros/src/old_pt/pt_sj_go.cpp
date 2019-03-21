#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <iostream>
#include <vector>
#include "iiwaRos.h"
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/Float64.h"
#include <fstream>
#include <string>
#include <stdio.h>
using namespace std;

float kx,ky,kz;
int pt_mode;
geometry_msgs::PoseStamped msg;
geometry_msgs::PoseStamped pose_comd;
geometry_msgs::PoseStamped pose_go;
geometry_msgs::WrenchStamped force_go;
std_msgs::Float64 pt_wrench_x;
std_msgs::Float64 pt_wrench_y;
std_msgs::Float64 pt_wrench_z;
void set_impedance(iiwa_msgs::ConfigureSmartServo& config1,int& pt);
void pose_chatterCallback(const geometry_msgs::PoseStamped& msg1)
{
  msg.pose.position.x=msg1.pose.position.x;
  msg.pose.position.y=msg1.pose.position.y;
  msg.pose.position.z=msg1.pose.position.z;
  msg.pose.orientation.x=msg1.pose.orientation.x;
  msg.pose.orientation.y=msg1.pose.orientation.y;
  msg.pose.orientation.z=msg1.pose.orientation.z;
  msg.pose.orientation.w=msg1.pose.orientation.w;

}
void pose_iiwa_Callback(const geometry_msgs::PoseStamped& msg1)
{
  pose_go.pose.position.x=msg1.pose.position.x;
  pose_go.pose.position.y=msg1.pose.position.y;
  pose_go.pose.position.z=msg1.pose.position.z;
  pose_go.pose.orientation.x =msg1.pose.orientation.x;
  pose_go.pose.orientation.y =msg1.pose.orientation.y;
  pose_go.pose.orientation.z =msg1.pose.orientation.z;
  pose_go.pose.orientation.w =msg1.pose.orientation.w;
}
void wrench_iiwa_callback(const geometry_msgs::WrenchStamped& msg1)
{
  force_go.wrench.force.x=msg1.wrench.force.x;
  force_go.wrench.force.y=msg1.wrench.force.y;
  force_go.wrench.force.z=msg1.wrench.force.z;
  force_go.wrench.torque.x=msg1.wrench.torque.x;
  force_go.wrench.torque.y=msg1.wrench.torque.y;
  force_go.wrench.torque.z=msg1.wrench.torque.z;
  
//ROS_INFO("listener_wrench:%f,%f,%f", msg1.wrench.force.x,msg1.wrench.force.y,msg1.wrench.force.z);
}
int main(int argc, char **argv)
{
 
  ros::init(argc, argv, "pt_sj_go");
  ros::NodeHandle n;
  ros::Subscriber pose_sub = n.subscribe("/pose", 100, pose_chatterCallback);
  ros::Publisher pose_command_pub = n.advertise<geometry_msgs::PoseStamped>("/iiwa/command/CartesianPose", 1000);
 ros::ServiceClient client = n.serviceClient<iiwa_msgs::ConfigureSmartServo>("/iiwa/configuration/configureSmartServo");
 ros::Subscriber pose_iiwa_sub = n.subscribe("/iiwa/state/CartesianPose", 1000, pose_iiwa_Callback);
 ros::Subscriber wrench_iiwa_sub = n.subscribe("/iiwa/state/CartesianWrench", 1000, wrench_iiwa_callback);
  ros::Publisher pose_go_pub = n.advertise<geometry_msgs::PoseStamped>("pose_go", 1000);
  ros::Publisher force_go_pub = n.advertise<geometry_msgs::WrenchStamped>("force_go", 1000);
  iiwa_msgs::ConfigureSmartServo config;
  ros::Rate loop_rate(256);
  pt_mode=3;
  ifstream kx_in("/home/edward/kuka_catkin/K/kx.txt");
  ifstream ky_in("/home/edward/kuka_catkin/K/ky.txt");
  ifstream kz_in("/home/edward/kuka_catkin/K/kz.txt");
  if(pt_mode!=3)
    {
    set_impedance(config,pt_mode);
    client.call(config);
    }
  while (ros::ok())
  {
  pose_comd.pose.position.x=msg.pose.position.x;
  pose_comd.pose.position.y=msg.pose.position.y;
  pose_comd.pose.position.z=msg.pose.position.z;
  pose_comd.pose.orientation.x=msg.pose.orientation.x;
  pose_comd.pose.orientation.y=msg.pose.orientation.y;
  pose_comd.pose.orientation.z=msg.pose.orientation.z;
  pose_comd.pose.orientation.w=msg.pose.orientation.w;
 
    if(pt_mode==3)
    {
         if(pose_comd.pose.position.z>0)
         {pose_command_pub.publish(pose_comd);
         kx_in>>kx;
         ky_in>>ky;
         kz_in>>kz;
         set_impedance(config,pt_mode);
         client.call(config);
         ROS_INFO("%f,%f,%f", kx,ky,kz);
         pose_go_pub.publish(pose_go);
         force_go_pub.publish(force_go);}

    }
    else
    {
         if(pose_comd.pose.position.z>0)
         {pose_command_pub.publish(pose_comd); 
          pose_go_pub.publish(pose_go);
          force_go_pub.publish(force_go);}
    }



    ros::spinOnce();

    loop_rate.sleep();
  }
  kx_in.close();
  ky_in.close();
  ky_in.close();


  return 0;
}
void set_impedance(iiwa_msgs::ConfigureSmartServo& config1,int& pt)
{
  if(pt==1){
  config1.request.mode.mode = iiwa_msgs::SmartServoMode::CARTESIAN_IMPEDANCE;
  config1.request.mode.cartesian_stiffness.stiffness.x = 200; 
  config1.request.mode.cartesian_stiffness.stiffness.y = 200; 
  config1.request.mode.cartesian_stiffness.stiffness.z = 200; 
  config1.request.mode.cartesian_stiffness.stiffness.a = 200;
  config1.request.mode.cartesian_stiffness.stiffness.b = 200; 
  config1.request.mode.cartesian_stiffness.stiffness.c = 200; 
  config1.request.mode.cartesian_damping.damping.x = 1; 
  config1.request.mode.cartesian_damping.damping.y = 1; 
  config1.request.mode.cartesian_damping.damping.z = 1; 
  config1.request.mode.cartesian_damping.damping.a = 1; 
  config1.request.mode.cartesian_damping.damping.b = 1; 
  config1.request.mode.cartesian_damping.damping.c = 1; 
  config1.request.mode.nullspace_stiffness =10; 
  config1.request.mode.nullspace_damping = 1;
  kx= config1.request.mode.cartesian_stiffness.stiffness.x;
  ky= config1.request.mode.cartesian_stiffness.stiffness.y;
  kz= config1.request.mode.cartesian_stiffness.stiffness.z;
  ROS_INFO("%f,%f,%f", kx,ky,kz);
   }
  else if(pt==2)
{
  config1.request.mode.mode = iiwa_msgs::SmartServoMode::CARTESIAN_IMPEDANCE;
  config1.request.mode.cartesian_stiffness.stiffness.x = 200; 
  config1.request.mode.cartesian_stiffness.stiffness.y = 200; 
  config1.request.mode.cartesian_stiffness.stiffness.z = 500; 
  config1.request.mode.cartesian_stiffness.stiffness.a = 200;
  config1.request.mode.cartesian_stiffness.stiffness.b = 200; 
  config1.request.mode.cartesian_stiffness.stiffness.c = 200; 
  config1.request.mode.cartesian_damping.damping.x = 1; 
  config1.request.mode.cartesian_damping.damping.y = 1; 
  config1.request.mode.cartesian_damping.damping.z = 1; 
  config1.request.mode.cartesian_damping.damping.a = 1; 
  config1.request.mode.cartesian_damping.damping.b = 1; 
  config1.request.mode.cartesian_damping.damping.c = 1; 
  config1.request.mode.nullspace_stiffness =10; 
  config1.request.mode.nullspace_damping = 1;
  kx= config1.request.mode.cartesian_stiffness.stiffness.x;
  ky= config1.request.mode.cartesian_stiffness.stiffness.y;
  kz= config1.request.mode.cartesian_stiffness.stiffness.z;
  ROS_INFO("%f,%f,%f", kx,ky,kz);
}
  else if(pt==3&&kz>0)
{
  config1.request.mode.mode = iiwa_msgs::SmartServoMode::CARTESIAN_IMPEDANCE;
  config1.request.mode.cartesian_stiffness.stiffness.x = kx; 
  config1.request.mode.cartesian_stiffness.stiffness.y = ky; 
  config1.request.mode.cartesian_stiffness.stiffness.z = kz; 
  config1.request.mode.cartesian_stiffness.stiffness.a = 200;
  config1.request.mode.cartesian_stiffness.stiffness.b = 200; 
  config1.request.mode.cartesian_stiffness.stiffness.c = 200; 
  config1.request.mode.cartesian_damping.damping.x = 1; 
  config1.request.mode.cartesian_damping.damping.y = 1; 
  config1.request.mode.cartesian_damping.damping.z = 1; 
  config1.request.mode.cartesian_damping.damping.a = 1; 
  config1.request.mode.cartesian_damping.damping.b = 1; 
  config1.request.mode.cartesian_damping.damping.c = 1; 
  config1.request.mode.nullspace_stiffness =10; 
  config1.request.mode.nullspace_damping = 1;
}
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
