#include "ros/ros.h"
#include "iiwaRos.h"
#include <cstdlib>
#include <iiwa_msgs/ConfigureSmartServo.h>
#include <iiwa_msgs/SmartServoMode.h>
#include "rosserial_arduino/Adc.h"

void set_impedance(iiwa_msgs::ConfigureSmartServo& config1,int& pt1);
void EMG_chatterCallback(const rosserial_arduino::Adc adc_msg);
void now_pose_callback(const geometry_msgs::PoseStamped& sub_msg0);
void set_pose(geometry_msgs::PoseStamped& pub_msg0);
int a=0;
int pt=1;
geometry_msgs::PoseStamped now_pose_msg,command_pose_msg;
int main(int argc, char **argv)
{
  ros::init(argc, argv, "pt_client");

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<iiwa_msgs::ConfigureSmartServo>("/iiwa/configuration/configureSmartServo");
  ros::Subscriber now_pose = n.subscribe("/iiwa/state/CartesianPose", 1000, now_pose_callback);
  ros::Publisher  command_pose = n.advertise<geometry_msgs::PoseStamped>("/iiwa/command/CartesianPose", 1000);
// ros::Subscriber sub=n.subscribe("adc", 1000, EMG_chatterCallback);
  iiwa_msgs::ConfigureSmartServo config;
  set_impedance(config,pt);
if (client.call(config)) {
    if(!config.response.success)
        ROS_ERROR_STREAM("Config failed, Java error: " << config.response.error);
    else
        ROS_INFO_STREAM("SmartServo Service successfully called.");
   }
 else {
    ROS_ERROR_STREAM("Config failed - service could not be called - QUITTING NOW !");
   }
  ros::Rate loop_rate(10);
  while (ros::ok()){
  set_pose(command_pose_msg);
  command_pose.publish(command_pose_msg);
  ros::spinOnce();
  loop_rate.sleep();
  }
 return 0;
  
}
void EMG_chatterCallback(const rosserial_arduino::Adc adc_msg)  
{

    ROS_INFO("%d", adc_msg.adc0);
    a = adc_msg.adc0;
    if(a<300)pt=0;
    else pt=1;
}
void set_impedance(iiwa_msgs::ConfigureSmartServo& config1,int& pt1)
{
 if(pt1==0){  
  config1.request.mode.mode = iiwa_msgs::SmartServoMode::CARTESIAN_IMPEDANCE;
 
  config1.request.mode.cartesian_stiffness.stiffness.x = 10; 
  config1.request.mode.cartesian_stiffness.stiffness.y = 10; 
  config1.request.mode.cartesian_stiffness.stiffness.z = 10; 
  config1.request.mode.cartesian_stiffness.stiffness.a =200;
  config1.request.mode.cartesian_stiffness.stiffness.b =200; 
  config1.request.mode.cartesian_stiffness.stiffness.c =200; 

  config1.request.mode.cartesian_damping.damping.x = 0.1; 
  config1.request.mode.cartesian_damping.damping.y = 0.1; 
  config1.request.mode.cartesian_damping.damping.z = 0.1; 
  config1.request.mode.cartesian_damping.damping.a = 0.1; 
  config1.request.mode.cartesian_damping.damping.b = 0.1; 
  config1.request.mode.cartesian_damping.damping.c = 0.1; 

  config1.request.mode.nullspace_stiffness = 10; 
  config1.request.mode.nullspace_damping = 0.3; 
  }
  else if(pt1==1) {
  config1.request.mode.mode = iiwa_msgs::SmartServoMode::CARTESIAN_IMPEDANCE;
  config1.request.mode.cartesian_stiffness.stiffness.x = 100; 
  config1.request.mode.cartesian_stiffness.stiffness.y = 100; 
  config1.request.mode.cartesian_stiffness.stiffness.z = 100; 
  config1.request.mode.cartesian_stiffness.stiffness.a = 200;
  config1.request.mode.cartesian_stiffness.stiffness.b = 200; 
  config1.request.mode.cartesian_stiffness.stiffness.c = 200; 

  config1.request.mode.cartesian_damping.damping.x = 1; 
  config1.request.mode.cartesian_damping.damping.y = 1; 
  config1.request.mode.cartesian_damping.damping.z = 0.5; 
  config1.request.mode.cartesian_damping.damping.a = 1; 
  config1.request.mode.cartesian_damping.damping.b = 1; 
  config1.request.mode.cartesian_damping.damping.c = 1; 

  config1.request.mode.nullspace_stiffness =10; 
  config1.request.mode.nullspace_damping = 1; 
  } 
  else if(pt1==2) {
  config1.request.mode.mode = iiwa_msgs::SmartServoMode::JOINT_IMPEDANCE;
  config1.request.mode.joint_stiffness.stiffness.a1 = 3000; 
  config1.request.mode.joint_stiffness.stiffness.a2 = 3000; 
  config1.request.mode.joint_stiffness.stiffness.a3 = 3000; 
  config1.request.mode.joint_stiffness.stiffness.a4 = 3000;
  config1.request.mode.joint_stiffness.stiffness.a5 = 3000; 
  config1.request.mode.joint_stiffness.stiffness.a6 = 0; 
  config1.request.mode.joint_stiffness.stiffness.a7 = 3000;

  config1.request.mode.joint_damping.damping.a1 = 0.7; 
  config1.request.mode.joint_damping.damping.a2 = 0.7; 
  config1.request.mode.joint_damping.damping.a3= 0.7; 
  config1.request.mode.joint_damping.damping.a4 = 0.7; 
  config1.request.mode.joint_damping.damping.a5 = 0.7; 
  config1.request.mode.joint_damping.damping.a6 = 0.1; 
  config1.request.mode.joint_damping.damping.a7 = 0.7;


  }
  else if(pt1==3) {
  config1.request.mode.mode = iiwa_msgs::SmartServoMode::JOINT_IMPEDANCE;
  config1.request.mode.joint_stiffness.stiffness.a1 = 100; 
  config1.request.mode.joint_stiffness.stiffness.a2 = 100; 
  config1.request.mode.joint_stiffness.stiffness.a3 = 100; 
  config1.request.mode.joint_stiffness.stiffness.a4 = 30;
  config1.request.mode.joint_stiffness.stiffness.a5 = 30; 
  config1.request.mode.joint_stiffness.stiffness.a6 = 30; 
  config1.request.mode.joint_stiffness.stiffness.a7 = 30;

  config1.request.mode.joint_damping.damping.a1 = 0.7; 
  config1.request.mode.joint_damping.damping.a2 = 0.7; 
  config1.request.mode.joint_damping.damping.a3 = 0.7; 
  config1.request.mode.joint_damping.damping.a4 = 0.7; 
  config1.request.mode.joint_damping.damping.a5 = 0.7; 
  config1.request.mode.joint_damping.damping.a6 = 0.7; 
  config1.request.mode.joint_damping.damping.a7 = 0.7;
  }

}
void now_pose_callback(const geometry_msgs::PoseStamped& sub_msg0)
{
  now_pose_msg.pose.position.x=sub_msg0.pose.position.x;
  now_pose_msg.pose.position.y=sub_msg0.pose.position.y;
  now_pose_msg.pose.position.z=sub_msg0.pose.position.z;
  ROS_INFO("now_pose:%f,%f,%f,%f,%f,%f,%f", sub_msg0.pose.position.x,sub_msg0.pose.position.y,sub_msg0.pose.position.z,sub_msg0.pose.orientation.x,sub_msg0.pose.orientation.y,sub_msg0.pose.orientation.z,sub_msg0.pose.orientation.w);

}
void set_pose(geometry_msgs::PoseStamped& pub_msg0)
{
     
    /* pub_msg0.pose.position.x=now_pose_msg.pose.position.x;
     pub_msg0.pose.position.y=now_pose_msg.pose.position.y;
     pub_msg0.pose.position.z=now_pose_msg.pose.position.z;*/
     pub_msg0.pose.position.x=0.42;
     pub_msg0.pose.position.y=0.02;
     pub_msg0.pose.position.z=0.7;

    pub_msg0.pose.orientation.x=-0.353685581886;
    pub_msg0.pose.orientation.y=0.935302317142;
    pub_msg0.pose.orientation.z=-0.00418645251225;
    pub_msg0.pose.orientation.w=0.0099287728899;
    ROS_INFO("command_pose:%f,%f,%f,%f,%f,%f,%f", pub_msg0.pose.position.x,pub_msg0.pose.position.y,pub_msg0.pose.position.z,pub_msg0.pose.orientation.x,pub_msg0.pose.orientation.y,pub_msg0.pose.orientation.z,pub_msg0.pose.orientation.w);

}
