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

float pose_pt1[7];
float kx,ky,kz,m;
int pt_mode,flg=1;
geometry_msgs::PoseStamped msg;
geometry_msgs::PoseStamped pose_comd;
geometry_msgs::PoseStamped pose_go;
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
  pt_wrench_x.data=msg1.wrench.force.x;
  pt_wrench_y.data=msg1.wrench.force.y;
  pt_wrench_z.data=msg1.wrench.force.z;
  
//ROS_INFO("listener_wrench:%f,%f,%f", msg1.wrench.force.x,msg1.wrench.force.y,msg1.wrench.force.z);
}
int main(int argc, char **argv)
{
 
  ros::init(argc, argv, "pt_go2");
  ros::NodeHandle n;
  ros::Subscriber pose_sub = n.subscribe("/pose", 100, pose_chatterCallback);
  ros::Publisher pose_command_pub = n.advertise<geometry_msgs::PoseStamped>("/iiwa/command/CartesianPose", 1000);
 ros::ServiceClient client = n.serviceClient<iiwa_msgs::ConfigureSmartServo>("/iiwa/configuration/configureSmartServo");
 ros::Subscriber pose_iiwa_sub = n.subscribe("/iiwa/state/CartesianPose", 1000, pose_iiwa_Callback);
 ros::Subscriber wrench_iiwa_sub = n.subscribe("/iiwa/state/CartesianWrench", 1000, wrench_iiwa_callback);
  ros::Publisher pose_go_pub = n.advertise<geometry_msgs::PoseStamped>("pose_go", 1000);
  ros::Publisher wrench_go_pubx = n.advertise<std_msgs::Float64>("force_x", 1000);
  ros::Publisher wrench_go_puby = n.advertise<std_msgs::Float64>("force_y", 1000);
  ros::Publisher wrench_go_pubz = n.advertise<std_msgs::Float64>("force_z", 1000);
  iiwa_msgs::ConfigureSmartServo config;
  ros::Rate loop_rate(256);
  pt_mode=3;
  ifstream kx_in("/home/edward/kuka_ws/K/k1.txt");
  ifstream ky_in("/home/edward/kuka_ws/K/k2.txt");
  ifstream kz1_in("/home/edward/kuka_ws/K/kx3.txt");
  ifstream kz2_in("/home/edward/kuka_ws/K/kx2.txt");
  ifstream kz3_in("/home/edward/kuka_ws/K/kx1.txt");
  if(pt_mode!=3)
    {
    set_impedance(config,pt_mode);
    client.call(config);
    ROS_INFO("%f,%f,%f", kx,ky,kz);
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
   m=m+1;
   if(m>=1&&m<=1000)
   {
    if(flg==1){
     ifstream kz1_in("/home/edward/kuka_ws/K/kx3.txt"); flg=2;}
     kz1_in>>kz;}
   if(m>=1000&&m<=2000)
   { 
   if(flg==2){
     ifstream kz2_in("/home/edward/kuka_ws/K/kx2.txt");flg=3;}
     kz2_in>>kz;}
   if(m>=2000&&m<=3000)
   {
        if(flg==3){ifstream kz3_in("/home/edward/kuka_ws/K/kx1.txt");flg=1;}
     kz3_in>>kz;}
   if(m>3000)
     {m=0; }
    kx_in>>kx;
    ky_in>>ky;
   
    if(pt_mode==3)
    {
    set_impedance(config,pt_mode);
    client.call(config);
    ROS_INFO("%f,%f,%f", kx,ky,kz);
     if(pose_comd.pose.position.z>0)
     {pose_command_pub.publish(pose_comd);

    }
     pose_go_pub.publish(pose_go);
    wrench_go_pubx.publish(pt_wrench_x);
    wrench_go_puby.publish(pt_wrench_y);
    wrench_go_pubz.publish(pt_wrench_z);
    //ROS_INFO("%f,%f,%f,%f,%f,%f,%f", msg.pose.position.x,msg.pose.position.y,msg.pose.position.z,msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w);
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
  config1.request.mode.cartesian_stiffness.stiffness.x = 1000; 
  config1.request.mode.cartesian_stiffness.stiffness.y = 1000; 
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
   }
  else if(pt==2)
{
  config1.request.mode.mode = iiwa_msgs::SmartServoMode::CARTESIAN_IMPEDANCE;
  config1.request.mode.cartesian_stiffness.stiffness.x = 50; 
  config1.request.mode.cartesian_stiffness.stiffness.y = 50; 
  config1.request.mode.cartesian_stiffness.stiffness.z = 50; 
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
  config1.request.mode.cartesian_damping.damping.z = 0.2; 
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
