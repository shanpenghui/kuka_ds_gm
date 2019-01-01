#include "ros/ros.h"
#include "iiwaRos.h"
#include <cstdlib>
#include <iiwa_msgs/ConfigureSmartServo.h>
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/Int16.h"
#include <iiwa_msgs/SmartServoMode.h>
#include "rosserial_arduino/Adc.h"

void set_impedance(iiwa_msgs::ConfigureSmartServo& config1,int& pt1);
void EMG_chatterCallback(const rosserial_arduino::Adc adc_msg);
int Adc0=0,Adc1=0;
int pt=1,flg=1,pt_flg=1,call_ready=1,pp=1;
std_msgs::Int16 pt_msg;
const int Threshold=2000;//阈值
int max_EMG=0;//峰值
int main(int argc, char **argv)
{
  ros::init(argc, argv, "pt_client");

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<iiwa_msgs::ConfigureSmartServo>("/iiwa/configuration/configureSmartServo");
  ros::Publisher EMG_pub = n.advertise<std_msgs::Int16>("EMGS", 1000);
  iiwa_msgs::ConfigureSmartServo config;
  ros::Subscriber sub=n.subscribe("adc", 1000, EMG_chatterCallback);
  ros::Rate loop_rate(10);
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
 
  EMG_pub.publish(pt_msg);
  ros::spinOnce();
  loop_rate.sleep();
 }
 return 0;
  
}
void EMG_chatterCallback(rosserial_arduino::Adc adc_msg)  
{

    Adc0= adc_msg.adc0;
    Adc1= adc_msg.adc1;


      max_EMG=Adc0>Adc1?Adc0:Adc1;
      if(max_EMG<Threshold)pt=0;
      else pt=1; 
     pt_msg.data=max_EMG;
    // ROS_INFO("%d,%d,%d", adc_msg.adc0,adc_msg.adc1,max_EMG);  
}
void set_impedance(iiwa_msgs::ConfigureSmartServo& config1,int& pt1)
{
 if(pt1==0&&pt_flg==0){  
  config1.request.mode.mode = iiwa_msgs::SmartServoMode::CARTESIAN_IMPEDANCE;
 
  config1.request.mode.cartesian_stiffness.stiffness.x = 1; 
  config1.request.mode.cartesian_stiffness.stiffness.y = 1; 
  config1.request.mode.cartesian_stiffness.stiffness.z = 1; 
  config1.request.mode.cartesian_stiffness.stiffness.a =1;
  config1.request.mode.cartesian_stiffness.stiffness.b =1; 
  config1.request.mode.cartesian_stiffness.stiffness.c =1; 

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
  pt_flg=0,call_ready=1;
  } 
  else if(pt1==2) {
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
  }
else{;}

}

