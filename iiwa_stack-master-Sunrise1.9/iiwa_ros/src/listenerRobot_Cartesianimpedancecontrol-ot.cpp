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
#include <Eigen/Dense>
#include <string>
//#include "funset.hpp"  //网上源码给的
//#include <string>  
//#include <opencv2/opencv.hpp>  
//#include "common.hpp" 

using namespace std;
using namespace Eigen;

//////////////////////定义变量/////////////////
int a = 0;
int b = 0;
float c = 0;
float d = 0;
int e = 0;
int f = 0;
int g = 0;
int h = 0;
int A1[200];
int B1[200];
int C1[200];
int D1[200];
float a1 = 0;
float b1 = 0;
float c1 = 0;
float d1 = 0;
float ha1 = 0;
float am = 0;
float bm = 0;
float cm = 0;
float dm = 0;
float cw = 0;
float ce = 0;
float tw = 0;
float te = 0;
int s = 0;
float r1 = 0;//笛卡尔位置
float r2 = 0;
float r3 = 0;
float r4 = 0;
float r5 = 0;
float r6 = 0;
float r7 = 0;
float t1 = 0;//关节力矩
float t2 = 0;
float t3 = 0;
float t4 = 0;
float t5 = 0;
float t6 = 0;
float t7 = 0;
float o1 = 0;//关节位置
float o2 = 0;
float o3 = 0;
float o4 = 0;
float o5 = 0;
float o6 = 0;
float o7 = 0;
float w1 = 0;//笛卡尔力
float w2 = 0;
float w3 = 0;
float w4 = 0;
float w5 = 0;
float w6 = 0;
MatrixXf Cd(6,6);
Matrix<float, 6, 7> J1;
Matrix<float, 6, 7> JT_aug;
Matrix<float, 7, 6> Jp;
Matrix<float, 7, 6> J2;
Matrix<float, 6, 7> Jpf;
Matrix<float, 7, 1> O;
Matrix<float, 7, 1> Oe;
Matrix<float, 6, 1> W;
Matrix<float, 7, 1>  T;
Matrix<float, 7, 1>  R;
Matrix<float, 7, 1>  Re;
Matrix<float, 6, 1>  V;
Matrix<float, 6, 1>  F;//U1(6,6)U2(6,6)x_dot(6,1)x_dotprev(6,1)q_dot(7,1)
Matrix<float, 6, 6> U1;
Matrix<float, 6, 6> U2;
Matrix<float, 6, 1> xdot;
Matrix<float, 6, 1> xdotprev;
Matrix<float, 7, 1> qdot;






void chatterCallback(const rosserial_arduino::Adc adc_msg)  //put EMG in a,b
{
// ROS_INFO("%d,%d", adc_msg.adc0,adc_msg.adc1);

    a = adc_msg.adc0;
    b = adc_msg.adc1;
    c = adc_msg.adc2;
    d = adc_msg.adc3;
}

void chatterCallback1(const geometry_msgs::PoseStamped& msg)  //put CartesianPose into r
{

//ROS_INFO("%f,%f,%f,%f,%f,%f,%f", msg.pose.position.x,msg.pose.position.y,msg.pose.position.z,msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w);


    r1 = msg.pose.position.x;
    r2 = msg.pose.position.y;
    r3 = msg.pose.position.z;
    r4 = msg.pose.orientation.x;
    r5 = msg.pose.orientation.y;
    r6 = msg.pose.orientation.z;
    r7 = msg.pose.orientation.w;  
    R[0]=r1;
    R[1]=r2;
    R[2]=r3;
    R[3]=r4;
    R[4]=r5;
    R[5]=r6;
    R[6]=r7;
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
    T[0]=t1;
    T[1]=t2;
    T[2]=t3;
    T[3]=t4;
    T[4]=t5;
    T[5]=t6;
    T[6]=t7;
//ROS_INFO("%f,%f,%f,%f,%f,%f,%f", msg.torque.a1,msg.torque.a2,msg.torque.a3,msg.torque.a4,msg.torque.a5,msg.torque.a6,msg.torque.a7);

} 

void chatterCallback3(const iiwa_msgs::JointPosition& msg) //put JointTorque into t
{

    o1 = msg.position.a1;
    o2 = msg.position.a2;
    o3 = msg.position.a3;
    o4 = msg.position.a4;
    o5 = msg.position.a5;
    o6 = msg.position.a6;
    o7 = msg.position.a7;   

    O[0]=o1;
    O[1]=o2;
    O[2]=o3;
    O[3]=o4;
    O[4]=o5;
    O[5]=o6;
    O[6]=o7;
//    ROS_INFO("%f,%f,%f,%f,%f,%f,%f", msg.position.a1,msg.position.a2,msg.position.a3,msg.position.a4,msg.position.a5,msg.position.a6,msg.position.a7);
} 

void chatterCallback4(const geometry_msgs::WrenchStamped& msg) //put JointTorque into t
{

    w1 = msg.wrench.force.x;
    w2 = msg.wrench.force.y;
    w3 = msg.wrench.force.z;
    w4 = msg.wrench.torque.x;
    w5 = msg.wrench.torque.y;
    w6 = msg.wrench.torque.z;

    W[0]=w1;
    W[1]=w2;
    W[2]=w3;
    W[3]=w4;
    W[4]=w5;
    W[5]=w6;
   
//    ROS_INFO("%f,%f,%f", msg.wrench.force.x,msg.wrench.force.y,msg.wrench.force.z);
} 


//////////////////主函数///////////////////////
int main(int argc, char **argv)
{
  ros::init(argc, argv, "listenerRobot");

  ros::NodeHandle n;

  ros::Subscriber sub1=n.subscribe("adc", 1000, chatterCallback);

  ros::Subscriber sub2=n.subscribe("/iiwa/state/CartesianPose", 1000, chatterCallback1);

  ros::Subscriber sub3=n.subscribe("/iiwa/state/JointTorque", 1000, chatterCallback2);

  ros::Subscriber sub4=n.subscribe("/iiwa/state/JointPosition", 1000, chatterCallback3);

  ros::Subscriber sub5=n.subscribe("/iiwa/state/CartesianWrench", 1000, chatterCallback4);

  ros::Publisher chatter_pub1 = n.advertise<std_msgs::Int16>("EMGs", 1000);

  ros::Publisher chatter_pub2 = n.advertise<geometry_msgs::PoseStamped>("/iiwa/command/CartesianPose", 1000);

//  ros::Publisher chatter_pub3 = n.advertise<iiwa_msgs::JointPosition>("/iiwa/command/JointPosition", 1000);

//  ros::Publisher chatter_pub4 = n.advertise<iiwa_msgs::JointVelocity>("/iiwa/command/JointVelocity", 1000);

  ros::Rate loop_rate(10);

am=1;//肌肉最大主动力
bm=1;

while (ros::ok())
 {

////////////////////////////////2肌电信号处理////////////////////////////

  std_msgs::Int16 msg;

a1=a/950.0;//归一化
b1=b/950.0;

ha1=200.0/950.0;//阈值设计

  if(a1>b1){//计算腕关节共收缩程度

    cw=a1;
    if(cw>ha1){//计算腕关节共收缩程度 
      s=1000;
       }
    else{
    s=20;
       }
   }
   else{
    cw=b1;
    if(cw>ha1) {//计算腕关节共收缩程度
    s=1000;
     }
    else{
    s=20;
     }
   }



  msg.data = s;

//  ROS_INFO("%d", msg.data);
 ROS_INFO("%f,%f", cw,ha1);
//ROS_INFO("%f,%f", a1,b1);
//ROS_INFO("%d,%d", c,d);
chatter_pub1.publish(msg);
//////////////////////////////////////////////////




  ros::spinOnce();

  loop_rate.sleep();

 }

  return 0;
}
