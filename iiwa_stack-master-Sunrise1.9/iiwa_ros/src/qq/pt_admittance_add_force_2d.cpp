#include "ros/ros.h"
#include "iiwaRos.h"
#include <cstdlib>
#include <iiwa_msgs/ConfigureSmartServo.h>
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Float32.h"
#include <iiwa_msgs/SmartServoMode.h>
#include "rosserial_arduino/Adc.h"

void set_impedance(iiwa_msgs::ConfigureSmartServo& config1,int& pt1);
void EMG_chatterCallback(const rosserial_arduino::Adc adc_msg);
void chatterCallback4(const geometry_msgs::WrenchStamped& msg); //put JointTorque into t
float Adc0=0,Adc1=0;
int pt=1,flg=1,pt_flg=1,call_ready=1;
float w1 = 0;//笛卡尔力
float w2 = 0;
float w3 = 0;

std_msgs::Int16 pt_msg;
const float Threshold=90/500.0;//阈值
float max_EMG=0;//峰值
float min_EMG=0;//对比中小值
int main(int argc, char **argv)
{
  ros::init(argc, argv, "pt_client");

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<iiwa_msgs::ConfigureSmartServo>("/iiwa/configuration/configureSmartServo");
  ros::Publisher EMG_pub = n.advertise<std_msgs::Int16>("EMGS", 1000);
  iiwa_msgs::ConfigureSmartServo config;
  ros::Subscriber sub=n.subscribe("adc", 1000, EMG_chatterCallback);
  ros::Publisher chatter_pub5 = n.advertise<std_msgs::Float32>("Fy", 1000);
  ros::Subscriber sub5=n.subscribe("/iiwa/state/CartesianWrench", 1000, chatterCallback4);
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
////////////////////////////////力信号处理////////////////////////////

  std_msgs::Float32 msg2;

  msg2.data = w1*50;

//  ROS_INFO("%d", msg.data);
// ROS_INFO("%f", w1);
//ROS_INFO("%f,%f", a1,b1);
//ROS_INFO("%d,%d", c,d);
chatter_pub5.publish(msg2);
//////////////////////////////////////////////////
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

void chatterCallback4(const geometry_msgs::WrenchStamped& msg) //put JointTorque into t
{

    w1 = msg.wrench.force.x;
    w2 = msg.wrench.force.y;
    w3 = msg.wrench.force.z;

//    t++;
//    F1[t]=w2;
//     if(t==50){
//      t=0;float sum_F=0;
//      for (int i = 0; i < 50; ++i) {
//      sum_F+=F1[i];
//       }
//     avr_F=sum_F/50;
//     }

//    ROS_INFO("%f,%f,%f", msg.wrench.force.x,msg.wrench.force.y,msg.wrench.force.z);
} 

void set_impedance(iiwa_msgs::ConfigureSmartServo& config1,int& pt1)
{
 if(pt1==0&&pt_flg==0){  
  config1.request.mode.mode = iiwa_msgs::SmartServoMode::CARTESIAN_IMPEDANCE;
 
  config1.request.mode.cartesian_stiffness.stiffness.x = 10; 
  config1.request.mode.cartesian_stiffness.stiffness.y = 1000; 
  config1.request.mode.cartesian_stiffness.stiffness.z = 30; 
  config1.request.mode.cartesian_stiffness.stiffness.a =50;
  config1.request.mode.cartesian_stiffness.stiffness.b =300; 
  config1.request.mode.cartesian_stiffness.stiffness.c =300; 

  config1.request.mode.cartesian_damping.damping.x = 0.1; 
  config1.request.mode.cartesian_damping.damping.y = 0.1; 
  config1.request.mode.cartesian_damping.damping.z = 0.1; 
  config1.request.mode.cartesian_damping.damping.a = 0.1; 
  config1.request.mode.cartesian_damping.damping.b = 0.1; 
  config1.request.mode.cartesian_damping.damping.c = 0.1; 

  config1.request.mode.nullspace_stiffness = 10; 
  config1.request.mode.nullspace_damping = 1; 
  pt_flg=1,call_ready=1;
  }
  else if(pt1==1&&pt_flg==1) {
  config1.request.mode.mode = iiwa_msgs::SmartServoMode::CARTESIAN_IMPEDANCE;
  config1.request.mode.cartesian_stiffness.stiffness.x = 20; 
  config1.request.mode.cartesian_stiffness.stiffness.y = 1000; 
  config1.request.mode.cartesian_stiffness.stiffness.z = 30; 
  config1.request.mode.cartesian_stiffness.stiffness.a = 50;
  config1.request.mode.cartesian_stiffness.stiffness.b = 300; 
  config1.request.mode.cartesian_stiffness.stiffness.c = 300; 

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
  config1.request.mode.joint_stiffness.stiffness.a5 = 3000; 
  config1.request.mode.joint_stiffness.stiffness.a6 = 0; 
  config1.request.mode.joint_stiffness.stiffness.a7 = 3000;

  config1.request.mode.joint_damping.damping.a1 = 0.7; 
  config1.request.mode.joint_damping.damping.a2 = 0.7; 
  config1.request.mode.joint_damping.damping.a3= 0.7; 
  config1.request.mode.joint_damping.damping.a4 = 0.7; 
  config1.request.mode.joint_damping.damping.a5 = 0.7; 
  config1.request.mode.joint_damping.damping.a6 = 0.3; 
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
  }*/
else{;}

}

