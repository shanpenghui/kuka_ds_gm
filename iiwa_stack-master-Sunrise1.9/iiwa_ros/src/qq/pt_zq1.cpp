#include "ros/ros.h"
#include "iiwaRos.h"
#include <cstdlib>
#include <iiwa_msgs/ConfigureSmartServo.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/WrenchStamped.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Float64.h"
#include <iiwa_msgs/SmartServoMode.h>
#include "rosserial_arduino/Adc.h"
#include "iostream"
#include "string"
#include "fstream"
void set_impedance(iiwa_msgs::ConfigureSmartServo& config1,int& pt1);
void EMG_chatterCallback(const rosserial_arduino::Adc adc_msg);
geometry_msgs::PoseStamped pose_msg;
geometry_msgs::PoseStamped pose_pt;
geometry_msgs::WrenchStamped force_pt;
float Adc0=0,Adc1=0;
int pt=1,flg=1,pt_flg=1,call_ready=1,pp=1;

std_msgs::Float64 pt_pose_x;
std_msgs::Float64 pt_pose_y;
std_msgs::Float64 pt_pose_z;
std_msgs::Float64 pt_pose_ox;
std_msgs::Float64 pt_pose_oy;
std_msgs::Float64 pt_pose_oz;
std_msgs::Float64 pt_pose_ow;
std_msgs::Float64 pt_wrench_x;
std_msgs::Float64 pt_wrench_y;
std_msgs::Float64 pt_wrench_z;
std_msgs::Int16 pt_msg;
const float Threshold=90/500.0;//阈值
float max_EMG=0;//峰值
float min_EMG=0;//对比中小值

void pose_chatterCallback(const geometry_msgs::PoseStamped& msg)
{
  pose_pt.pose.position.x=msg.pose.position.x;
  pose_pt.pose.position.y=msg.pose.position.y;
  pose_pt.pose.position.z=msg.pose.position.z;
  pose_pt.pose.orientation.x =msg.pose.orientation.x;
  pose_pt.pose.orientation.y =msg.pose.orientation.y;
  pose_pt.pose.orientation.z =msg.pose.orientation.z;
  pose_pt.pose.orientation.w =msg.pose.orientation.w;
ROS_INFO("listener_pose:%f,%f,%f", pose_pt.pose.position.x,pose_pt.pose.position.y,pose_pt.pose.position.z);
}
void wrench_callback(const geometry_msgs::WrenchStamped& msg1)
{
  force_pt.wrench.force.x=msg1.wrench.force.x;
  force_pt.wrench.force.y=msg1.wrench.force.y;
  force_pt.wrench.force.z=msg1.wrench.force.z;
  force_pt.wrench.torque.x=msg1.wrench.torque.x;
  force_pt.wrench.torque.y=msg1.wrench.torque.y;
  force_pt.wrench.torque.z=msg1.wrench.torque.z;
  
}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "pt_zq");

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<iiwa_msgs::ConfigureSmartServo>("/iiwa/configuration/configureSmartServo");
  ros::Subscriber Adc_sub=n.subscribe("adc", 1000, EMG_chatterCallback);
  ros::Subscriber pose_sub = n.subscribe("/iiwa/state/CartesianPose", 1000, pose_chatterCallback);
  ros::Publisher EMG_pub = n.advertise<std_msgs::Int16>("sEMG", 1000);
  ros::Subscriber wrench_sub = n.subscribe("/iiwa/state/CartesianWrench", 1000, wrench_callback);
  ros::Publisher pose_pub = n.advertise<geometry_msgs::PoseStamped>("pose", 1000);
  ros::Publisher force_pub = n.advertise<geometry_msgs::WrenchStamped>("force", 1000);
  iiwa_msgs::ConfigureSmartServo config;
  ros::Rate loop_rate(100);
  while (ros::ok()){
  set_impedance(config,pt);
 
   if(call_ready==1) 
   if (client.call(config)) {
       if(!config.response.success)
        ROS_ERROR_STREAM("Config failed, Java error: " << config.response.error);
       else {
               if(pt==0&&flg==0){
               ROS_INFO_STREAM("Low impedance successfully set.");flg=1;call_ready=0;}
               if(pt==1&&flg==1){
              ROS_INFO_STREAM("High impedance successfully set.");flg=0;call_ready=0;}
            }  
    }
  else {
    ROS_ERROR_STREAM("Config failed - service could not be called - QUITTING NOW !");
     }
   pose_pub.publish(pose_pt);
  force_pub.publish(force_pt);
  EMG_pub.publish(pt_msg);
  ros::spinOnce();
  loop_rate.sleep();
 }
 return 0;
  
}
void EMG_chatterCallback(rosserial_arduino::Adc adc_msg)  
{

    Adc0= adc_msg.adc0/600.0;
    Adc1= adc_msg.adc1/500.0;


//      max_EMG=Adc0>Adc1?Adc0:Adc1;//最大值作为评定标准
//      if(max_EMG<Threshold){pt=0;pt_msg.data=0;}
//      else {pt=1;pt_msg.data=1000;} 
//    // ROS_INFO("%d,%d,%d", adc_msg.adc0,adc_msg.adc1,max_EMG);  

      min_EMG=Adc0<Adc1?Adc0:Adc1;//最小值作为评定标准
      if(min_EMG<Threshold){pt=0;pt_msg.data=0;}//pt=0;pt_msg.data=0;
      else {pt=1;pt_msg.data=1000;} //pt=1;pt_msg.data=1000;
     ROS_INFO("%f,%f,%f,%f", Adc0,Adc1,min_EMG,Threshold);  //,adc_msg.adc1,min_EMG
}
void set_impedance(iiwa_msgs::ConfigureSmartServo& config1,int& pt1)
{
 if(pt1==0&&pt_flg==0){  
  config1.request.mode.mode = iiwa_msgs::SmartServoMode::CARTESIAN_IMPEDANCE;
 
  config1.request.mode.cartesian_stiffness.stiffness.x = 10; 
  config1.request.mode.cartesian_stiffness.stiffness.y = 10; 
  config1.request.mode.cartesian_stiffness.stiffness.z = 20; 
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
  pt_flg=1,call_ready=1;
  }
  else if(pt1==1&&pt_flg==1) {
  config1.request.mode.mode = iiwa_msgs::SmartServoMode::CARTESIAN_IMPEDANCE;
  config1.request.mode.cartesian_stiffness.stiffness.x = 20; 
  config1.request.mode.cartesian_stiffness.stiffness.y = 20; 
  config1.request.mode.cartesian_stiffness.stiffness.z = 30; 
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
  pt_flg=0,call_ready=1;
  } 
 /* else if(pt1==2) {
  config1.request.mode.mode = iiwa_msgs::SmartServoMode::JOINT_IMPEDANCE;
  config1.request.mode.joint_stiffness.stiffness.a1 = 3000; 
  config1.request.mode.joint_stiffness.stiffness.a2 = 3000; 
  config1.request.mode.joint_stiffness.stiffness.a3 = 3000; 
  config1.request.mode.joint_stiffness.stiffness.a4 = 3000;
  config1.request.mode.joint_stiffness.stiffness.a5 = 1; 
  config1.request.mode.joint_stiffness.stiffness.a6 = 1; 
  config1.request.mode.joint_stiffness.stiffness.a7 = 1;

  config1.request.mode.joint_damping.damping.a1 = 0.7; 
  config1.request.mode.joint_damping.damping.a2 = 0.7; 
  config1.request.mode.joint_damping.damping.a3= 0.7; 
  config1.request.mode.joint_damping.damping.a4 = 0.7; 
  config1.request.mode.joint_damping.damping.a5 = 0.1; 
  config1.request.mode.joint_damping.damping.a6 = 0.1; 
  config1.request.mode.joint_damping.damping.a7 = 0.1;


  }
  else if(pt1==3) {
  config1.request.mode.mode = iiwa_msgs::SmartServoMode::JOINT_IMPEDANCE;
  config1.request.mode.joint_stiffness.stiffness.a1 = 3000; 
  config1.request.mode.joint_stiffness.stiffness.a2 = 3000; 
  config1.request.mode.joint_stiffness.stiffness.a3 = 3000; 
  config1.request.mode.joint_stiffness.stiffness.a4 = 3000;
  config1.request.mode.joint_stiffness.stiffness.a5 = 10; 
  config1.request.mode.joint_stiffness.stiffness.a6 = 10; 
  config1.request.mode.joint_stiffness.stiffness.a7 = 10;

  config1.request.mode.joint_damping.damping.a1 = 0.7; 
  config1.request.mode.joint_damping.damping.a2 = 0.7; 
  config1.request.mode.joint_damping.damping.a3 = 0.7; 
  config1.request.mode.joint_damping.damping.a4 = 0.7; 
  config1.request.mode.joint_damping.damping.a5 = 0.7; 
  config1.request.mode.joint_damping.damping.a6 = 0.7; 
  config1.request.mode.joint_damping.damping.a7 = 0.7;
  }*/
else{;}

}

