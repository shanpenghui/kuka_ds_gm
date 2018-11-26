#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float32.h"
#include "rosserial_arduino/Adc.h"
#include <iiwa_ros.h>
#include "geometry_msgs/PoseStamped.h"
#include <iostream>  
#include "math.h"  
#include <sstream>
#include <vector>
#include "Eigen/Eigen"

using namespace std;

int a = 0;
int b = 0;
int c = 0;
int d = 0;
float r1 = 0;
float r2 = 0;
float r3 = 0;
float r4 = 0;
float r5 = 0;
float r6 = 0;
float r7 = 0;
float t1 = 0;
float t2 = 0;
float t3 = 0;
float t4 = 0;
float t5 = 0;
float t6 = 0;
float t7 = 0;
int A1[20];
int B1[20];

void chatterCallback(const rosserial_arduino::Adc adc_msg)  //put EMG in a,b
{

// ROS_INFO("%d,%d", adc_msg.adc0,adc_msg.adc1);

    a = adc_msg.adc0;
    b = adc_msg.adc1;
}

void chatterCallback1(const geometry_msgs::PoseStamped& msg)  //put CartesianPose into r
{

ROS_INFO("%f,%f,%f,%f,%f,%f,%f", msg.pose.position.x,msg.pose.position.y,msg.pose.position.z,msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w);


    r1 = msg.pose.position.x;
    r2 = msg.pose.position.y;
    r3 = msg.pose.position.z;
    r4 = msg.pose.orientation.x;
    r5 = msg.pose.orientation.y;
    r6 = msg.pose.orientation.z;
    r7 = msg.pose.orientation.w;  
}


void chatterCallback2(const iiwa_msgs::JointTorque& msg) //put JointTorque into t
{
//ROS_INFO("%f,%f,%f,%f,%f,%f,%f", msg.torque.a1,msg.torque.a2,msg.torque.a3,msg.torque.a4,msg.torque.a5,msg.torque.a6,msg.torque.a7);


    t1 = msg.torque.a1;
    t2 = msg.torque.a2;
    t3 = msg.torque.a3;
    t4 = msg.torque.a4;
    t5 = msg.torque.a5;
    t6 = msg.torque.a6;
    t7 = msg.torque.a7;   
//    tc=Jagebizhuanzhi(t1,t2,t3,t4,t5,t6,t7);
} 

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listenerRobot");

  ros::NodeHandle n;

  ros::Subscriber sub1=n.subscribe("adc", 1000, chatterCallback);

  ros::Subscriber sub2=n.subscribe("/iiwa/state/CartesianPose", 1000, chatterCallback1);

  ros::Subscriber sub3=n.subscribe("/iiwa/state/JointTorque", 1000, chatterCallback2);

  ros::Publisher chatter_pub1 = n.advertise<std_msgs::Int16>("EMGs", 1000);

  ros::Publisher chatter_pub2 = n.advertise<geometry_msgs::PoseStamped>("/iiwa/command/CartesianPose", 1000);

  ros::Rate loop_rate(10);

 int count = 0;
while (ros::ok())
 {
  std_msgs::Int16 msg;
    geometry_msgs::PoseStamped msg1;

  msg.data = a+b;
  ROS_INFO("%d", msg.data);

  chatter_pub1.publish(msg);

    msg1.pose.position.x=0.4;
    msg1.pose.position.y=0.02;
    msg1.pose.position.z=0.8;

    msg1.pose.orientation.x=-0.002456;
    msg1.pose.orientation.y=0.004966;
    msg1.pose.orientation.z=-0.306637;
    msg1.pose.orientation.w=0.951810;
   ROS_INFO("%f,%f,%f", msg1.pose.position.x,msg1.pose.position.y,msg1.pose.position.z);



  chatter_pub2.publish(msg1);

  ros::spinOnce();

  loop_rate.sleep();
    ++count;
 }

  return 0;
}
