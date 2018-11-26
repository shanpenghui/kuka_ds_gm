#include "ros/ros.h"//本程序就是根据性能约束的文章做出的末端三维方向阻抗控制器，可以实现末端的简单阻抗控制，但是效果并不是很好，机械臂有小幅的抖动，而且ｚ方向会向下漂移，这个问题可能是源于１，信号的漂移，２，可能存在控制模型内部的问题。
#include "std_msgs/String.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float32.h"
#include "rosserial_arduino/Adc.h"
#include "geometry_msgs/PoseStamped.h"
#include <iostream>  
#include "math.h"  
#include <sstream>
#include "Eigen/Eigen"
#include <Eigen/Dense>
#include <vector>
#include <string>
#include "iiwaRos.h"//clint的头文件
#include <cstdlib>
#include <iiwa_msgs/ConfigureSmartServo.h>
#include <iiwa_msgs/SmartServoMode.h>

using namespace std;
using namespace Eigen;

//////////////////////定义变量/////////////////
int a = 0;//肌电信号的四个初始值和四个滤波值
int b = 0;
int t = 0;
float c = 0;
float d = 0;
int e = 0;
int f = 0;
int g = 0;
int h = 0;
int A1[200];//肌电信号滤波向量
int B1[200];
int C1[200];
int D1[200];
float F1[50];//肌电信号滤波向量
float F2[100];
float F3[20];
float F4[20];
float avr_F = 0;
float f1 = 0;
float f2 = 0;
float a1 = 0;//肌电信号的百分比值
float b1 = 0;
float c1 = 0;
float d1 = 0;
float ha1 = 0;
float am = 0;//肌电信号最大主动力值
float bm = 0;
float cm = 0;
float dm = 0;
float cw = 0;//肌肉共收缩值
float ce = 0;
float tw = 0;//肌肉共收缩阈值
float te = 0;
int s = 0;//刚度值
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
MatrixXf Cd(6,6);//定义刚度矩阵
MatrixXf Md(6,6);
Matrix<float, 6, 7> J1;//雅格比
Matrix<float, 7, 6> J2;//雅格比转置
Matrix<float, 6, 7> Jpf;//雅格比转置后的伪逆
Matrix<float, 7, 1> O;//关节量
Matrix<float, 7, 1> Oe;
Matrix<float, 6, 1> W;//笛卡尔力
Matrix<float, 7, 1>  T;//关节力矩向量
Matrix<float, 7, 1>  R;//笛卡尔位置向量
Matrix<float, 7, 1>  Re;//笛卡尔位置向量
Matrix<float, 6, 1>  V;

Matrix<float,7, 3> JR_p;
Matrix<float, 7, 3> Jp;//雅格比伪逆


////////////////////子函数////////////////////////////
int calcRMS(int Data[], int Num)  //EMG processing
{  
    int fSum = 0;  
    for (int i = 0; i < Num; ++i)  
    {  
        fSum += Data[i];  
    }  

    //return sqrt(fSum/Num); 
    return fSum/Num; 
}

float calcRMS1(float Data[], int Num)  //F processing
{  
    float fSum = 0;  
    for (int i = 0; i < Num; ++i)  
    {  
        fSum += Data[i];  
    }  

    //return sqrt(fSum/Num); 
    return fSum/Num; 
}




//////////////////////回调函数////////



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



void chatterCallback4(const geometry_msgs::WrenchStamped& msg) //put JointTorque into t
{

    w1 = msg.wrench.force.x;
    w2 = msg.wrench.force.y;
    w3 = msg.wrench.force.z;
    w4 = msg.wrench.torque.x;
    w5 = msg.wrench.torque.y;
    w6 = msg.wrench.torque.z;

    W[0]=w1;
    W[1]=-w2;
    W[2]=-w3;
    W[3]=w4;
    W[4]=w5;
    W[5]=w6;
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

//////////////////主函数///////////////////////
int main(int argc, char **argv)
{
  ros::init(argc, argv, "listenerRobot");

  ros::NodeHandle n;

  ros::Subscriber sub2=n.subscribe("/iiwa/state/CartesianPose", 1000, chatterCallback1);

  ros::Subscriber sub5=n.subscribe("/netft_data", 1000, chatterCallback4);

  ros::Publisher chatter_pub3 = n.advertise<geometry_msgs::PoseStamped>("/iiwa/command/CartesianPose", 1000);//n.advertise<iiwa_msgs::JointPosition>("/iiwa/command/JointPosition", 1000);

  ros::Rate loop_rate(140);

//阻抗初始化
Matrix<float, 3, 3> U1;
Matrix<float, 3, 3> U2;
Matrix<float, 3, 1> xdot;
Matrix<float, 3, 1> xdotprev;
Matrix<float, 3, 1> x;
Matrix<float, 7, 1> qdot;
Matrix<int, 3, 1> fq;
Matrix<float, 3, 1> f_2;
Matrix<float, 3, 1> f_1;
float Ts=0.02;//iiwa控制环频率
xdot.fill(0);
xdotprev.fill(0);
Cd=MatrixXf::Zero(3,3); 
Md=MatrixXf::Zero(3,3); 
Matrix<float,3, 3> Cdd; 
Matrix<float,3, 3> Mdd; 


Cdd.setIdentity(3,3);
Cd=7*Cdd;

Mdd.setIdentity(3,3);
Md=0.001*Cdd;

Matrix<float,3, 7> JT; 
Matrix<float,3, 7> JR; 
Matrix<float,3, 7> JT_aug;
Matrix<float,7, 7> eye;
eye.setIdentity(7,7);
float K=0.3;

am=1;//肌肉最大主动力
bm=1;

while (ros::ok())
 {

//////////////////////////////////力信号处理////////////////////////////

//  std_msgs::Int16 msg;

//  for (int i = 0; i < 200; ++i) 
//{
//    if(i<199)
//    {
//    A1[i]=a;B1[i]=b;
//    }
//    else
//    {
//     A1[i]=a; c = calcRMS(A1, 200); B1[i]=b; d = calcRMS(B1, 200);
//    }
//}


//  msg.data = s;

////  ROS_INFO("%d", msg.data);
// ROS_INFO("%f,%f", cw,ha1);
////ROS_INFO("%f,%f", a1,b1);
////ROS_INFO("%d,%d", c,d);
//chatter_pub1.publish(msg);
////////////////////////////////////////////////////


////////////////////////////////关节导纳控制器(三方向)//////////////////////////////////

    geometry_msgs::PoseStamped msg1;  


  for (int i = 0; i < 3; ++i) //消去力的震动
{   
    W(i)=W(i)*10;
    fq(i)=W(i);
    f_2(i)=fq(i);
    f_2(i)=f_2(i)/10;
}

	
/*
  for (int i = 0; i < 3; ++i) //消去力的震动
{   
    f_2(i)=W(i);
}*/

f_2=f_1+K*(f_2-f_1);
    U1=Md/Ts + Cd;
    U2=U1.inverse();
    xdot=U2*(Md*xdotprev/Ts+f_2);  //如果只求到这一步，输出的位置是很稳的
f_1=f_2;

  x(0)=Ts*xdot(0)*(-1)+R(0);
  x(1)=Ts*xdot(1)*(-1)+R(1);
  x(2)=Ts*xdot(2)+R(2);

  xdotprev=xdot;

  msg1.pose.position.x=x(0);
  msg1.pose.position.y=x(1);
  msg1.pose.position.z=x(2);
  msg1.pose.orientation.x=-0.353685581886;
  msg1.pose.orientation.y=0.935302317142;
  msg1.pose.orientation.z=-0.00418645251225;
  msg1.pose.orientation.w=0.00992877288998;

if(x(0)!=0)
{chatter_pub3.publish(msg1);}
 
 // x(0)=0.421120645169;x(1)=0.0205836859991;x(2)=0.581226265672;

//   ROS_INFO("%f,%f,%f,%f,%f,%f", Jpf(0,0),Jpf(0,1),Jpf(0,2),Jpf(0,3),Jpf(0,4),Jpf(0,5));
//   ROS_INFO("%f,%f,%f,%f,%f,%f", Jp(0,0),Jp(0,1),Jp(0,2),Jp(0,3),Jp(0,4),Jp(0,5));
//   ROS_INFO("%f,%f,%f,%f,%f,%f", J1(0,0),J1(0,1),J1(0,2),J1(0,3),J1(0,4),J1(0,5));
  // ROS_INFO("%f,%f,%f,%f,%f,%f", qdot(0),qdot(1),qdot(2),qdot(3),qdot(4),qdot(5));
// ROS_INFO("%f,%f,%f", xdot(0),xdot(1),xdot(2));
ROS_INFO("%f,%f,%f", x(0),x(1),x(2));
//ROS_INFO("%f,%f,%f", R(0),R(1),R(2));


/////////////////////////////////////////////////////////////////

  ros::spinOnce();

  loop_rate.sleep();

 }

  return 0;
}
