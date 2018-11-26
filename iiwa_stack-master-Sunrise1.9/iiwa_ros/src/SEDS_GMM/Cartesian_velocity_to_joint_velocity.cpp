#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float32.h"
#include "rosserial_arduino/Adc.h"
#include "geometry_msgs/PoseStamped.h"
#include <iostream>  
#include "math.h"  
#include <sstream>
#include <vector>
#include "Eigen/Eigen"
#include <Eigen/Dense>
#include <string>
#include <Eigen/Core> 
#include <Eigen/SVD>   

#include "iiwaRos.h"//clint的头文件
#include <cstdlib>
#include <iiwa_msgs/ConfigureSmartServo.h>
#include <iiwa_msgs/SmartServoMode.h>

using namespace std;
using namespace Eigen;

//////////////////////定义变量/////////////////
float r1 = 0;//笛卡尔位置
float r2 = 0;
float r3 = 0;
float r4 = 0;
float r5 = 0;
float r6 = 0;
float r7 = 0;

float o1 = 0.1;//关节位置
float o2 = 0.2;
float o3 = 0.3;
float o4 = 0.2;
float o5 = 0.2;
float o6 = 0.3;
float o7 = 0.2;

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


void Jg(Eigen::Matrix<float,7,1> Data, Eigen::Matrix<float,6,7> J)  
{  
    o1=Data[0];o2=Data[1];o3=Data[2];o4=Data[3];o5=Data[4];o6=Data[5];o7=Data[6];
    J(0,0)= (21*sin(o1)*sin(o2))/50 - (63*cos(o6)*(sin(o4)*(cos(o1)*sin(o3) + cos(o2)*cos(o3)*sin(o1)) - cos(o4)*sin(o1)*sin(o2)))/500 + (63*sin(o6)*(cos(o5)*(cos(o4)*(cos(o1)*sin(o3) + cos(o2)*cos(o3)*sin(o1)) + sin(o1)*sin(o2)*sin(o4)) + sin(o5)*(cos(o1)*cos(o3) - cos(o2)*sin(o1)*sin(o3))))/500 - (2*sin(o4)*(cos(o1)*sin(o3) + cos(o2)*cos(o3)*sin(o1)))/5 + (2*cos(o4)*sin(o1)*sin(o2))/5;
    J(1,0)= (63*sin(o6)*(cos(o5)*(cos(o4)*(sin(o1)*sin(o3) - cos(o1)*cos(o2)*cos(o3)) - cos(o1)*sin(o2)*sin(o4)) + sin(o5)*(cos(o3)*sin(o1) + cos(o1)*cos(o2)*sin(o3))))/500 - (63*cos(o6)*(sin(o4)*(sin(o1)*sin(o3) - cos(o1)*cos(o2)*cos(o3)) + cos(o1)*cos(o4)*sin(o2)))/500 - (21*cos(o1)*sin(o2))/50 - (2*sin(o4)*(sin(o1)*sin(o3) - cos(o1)*cos(o2)*cos(o3)))/5 - (2*cos(o1)*cos(o4)*sin(o2))/5;
    J(2,0)= 0;
    J(3,0)= 0;
    J(4,0)= 0;
    J(5,0)= 1;
    J(0,1)= - (21*cos(o1)*cos(o2))/50 - (63*cos(o6)*(cos(o1)*cos(o2)*cos(o4) + cos(o1)*cos(o3)*sin(o2)*sin(o4)))/500 - (63*sin(o6)*(cos(o5)*(cos(o1)*cos(o2)*sin(o4) - cos(o1)*cos(o3)*cos(o4)*sin(o2)) + cos(o1)*sin(o2)*sin(o3)*sin(o5)))/500 - (2*cos(o1)*cos(o2)*cos(o4))/5 - (2*cos(o1)*cos(o3)*sin(o2)*sin(o4))/5;
    J(1,1)= - (21*cos(o2)*sin(o1))/50 - (63*cos(o6)*(cos(o2)*cos(o4)*sin(o1) + cos(o3)*sin(o1)*sin(o2)*sin(o4)))/500 - (63*sin(o6)*(cos(o5)*(cos(o2)*sin(o1)*sin(o4) - cos(o3)*cos(o4)*sin(o1)*sin(o2)) + sin(o1)*sin(o2)*sin(o3)*sin(o5)))/500 - (2*cos(o2)*cos(o4)*sin(o1))/5 - (2*cos(o3)*sin(o1)*sin(o2)*sin(o4))/5;
    J(2,1)= (2*cos(o2)*cos(o3)*sin(o4))/5 - (2*cos(o4)*sin(o2))/5 - (63*sin(o6)*(cos(o5)*(sin(o2)*sin(o4) + cos(o2)*cos(o3)*cos(o4)) - cos(o2)*sin(o3)*sin(o5)))/500 - (63*cos(o6)*(cos(o4)*sin(o2) - cos(o2)*cos(o3)*sin(o4)))/500 - (21*sin(o2))/50;
    J(3,1)= sin(o1);
    J(4,1)= -cos(o1);
    J(5,1)= 0;
    J(0,2)=  - (2*sin(o4)*(cos(o3)*sin(o1) + cos(o1)*cos(o2)*sin(o3)))/5 - (63*sin(o6)*(sin(o5)*(sin(o1)*sin(o3) - cos(o1)*cos(o2)*cos(o3)) - cos(o4)*cos(o5)*(cos(o3)*sin(o1) + cos(o1)*cos(o2)*sin(o3))))/500 - (63*cos(o6)*sin(o4)*(cos(o3)*sin(o1) + cos(o1)*cos(o2)*sin(o3)))/500;
    J(1,2)= (2*sin(o4)*(cos(o1)*cos(o3) - cos(o2)*sin(o1)*sin(o3)))/5 + (63*sin(o6)*(sin(o5)*(cos(o1)*sin(o3) + cos(o2)*cos(o3)*sin(o1)) - cos(o4)*cos(o5)*(cos(o1)*cos(o3) - cos(o2)*sin(o1)*sin(o3))))/500 + (63*cos(o6)*sin(o4)*(cos(o1)*cos(o3) - cos(o2)*sin(o1)*sin(o3)))/500;
    J(2,2)= (63*sin(o6)*(cos(o3)*sin(o2)*sin(o5) + cos(o4)*cos(o5)*sin(o2)*sin(o3)))/500 - (2*sin(o2)*sin(o3)*sin(o4))/5 - (63*cos(o6)*sin(o2)*sin(o3)*sin(o4))/500;
    J(3,2)= -cos(o1)*sin(o2);
    J(4,2)= -sin(o1)*sin(o2);
    J(5,2)=  cos(o2);
    J(0,3)= (2*cos(o1)*sin(o2)*sin(o4))/5 - (2*cos(o4)*(sin(o1)*sin(o3) - cos(o1)*cos(o2)*cos(o3)))/5 - (63*cos(o6)*(cos(o4)*(sin(o1)*sin(o3) - cos(o1)*cos(o2)*cos(o3)) - cos(o1)*sin(o2)*sin(o4)))/500 - (63*cos(o5)*sin(o6)*(sin(o4)*(sin(o1)*sin(o3) - cos(o1)*cos(o2)*cos(o3)) + cos(o1)*cos(o4)*sin(o2)))/500;
    J(1,3)= (63*cos(o6)*(cos(o4)*(cos(o1)*sin(o3) + cos(o2)*cos(o3)*sin(o1)) + sin(o1)*sin(o2)*sin(o4)))/500 + (2*cos(o4)*(cos(o1)*sin(o3) + cos(o2)*cos(o3)*sin(o1)))/5 + (2*sin(o1)*sin(o2)*sin(o4))/5 + (63*cos(o5)*sin(o6)*(sin(o4)*(cos(o1)*sin(o3) + cos(o2)*cos(o3)*sin(o1)) - cos(o4)*sin(o1)*sin(o2)))/500;
    J(2,3)=  (63*cos(o5)*sin(o6)*(cos(o2)*cos(o4) + cos(o3)*sin(o2)*sin(o4)))/500 - (63*cos(o6)*(cos(o2)*sin(o4) - cos(o3)*cos(o4)*sin(o2)))/500 - (2*cos(o2)*sin(o4))/5 + (2*cos(o3)*cos(o4)*sin(o2))/5;
    J(3,3)=  - cos(o3)*sin(o1) - cos(o1)*cos(o2)*sin(o3);
    J(4,3)=  cos(o1)*cos(o3) - cos(o2)*sin(o1)*sin(o3);
    J(5,3)= -sin(o2)*sin(o3);
    J(0,4)= -(63*sin(o6)*(sin(o5)*(cos(o4)*(sin(o1)*sin(o3) - cos(o1)*cos(o2)*cos(o3)) - cos(o1)*sin(o2)*sin(o4)) - cos(o5)*(cos(o3)*sin(o1) + cos(o1)*cos(o2)*sin(o3))))/500;
    J(1,4)= (63*sin(o6)*(sin(o5)*(cos(o4)*(cos(o1)*sin(o3) + cos(o2)*cos(o3)*sin(o1)) + sin(o1)*sin(o2)*sin(o4)) - cos(o5)*(cos(o1)*cos(o3) - cos(o2)*sin(o1)*sin(o3))))/500;
    J(2,4)= -(63*sin(o6)*(sin(o5)*(cos(o2)*sin(o4) - cos(o3)*cos(o4)*sin(o2)) - cos(o5)*sin(o2)*sin(o3)))/500;
    J(3,4)=  - sin(o4)*(sin(o1)*sin(o3) - cos(o1)*cos(o2)*cos(o3)) - cos(o1)*cos(o4)*sin(o2);
    J(4,4)= sin(o4)*(cos(o1)*sin(o3) + cos(o2)*cos(o3)*sin(o1)) - cos(o4)*sin(o1)*sin(o2);
    J(5,4)= cos(o2)*cos(o4) + cos(o3)*sin(o2)*sin(o4);
    J(0,5)=  (63*sin(o6)*(sin(o4)*(sin(o1)*sin(o3) - cos(o1)*cos(o2)*cos(o3)) + cos(o1)*cos(o4)*sin(o2)))/500 + (63*cos(o6)*(cos(o5)*(cos(o4)*(sin(o1)*sin(o3) - cos(o1)*cos(o2)*cos(o3)) - cos(o1)*sin(o2)*sin(o4)) + sin(o5)*(cos(o3)*sin(o1) + cos(o1)*cos(o2)*sin(o3))))/500;
    J(1,5)= - (63*sin(o6)*(sin(o4)*(cos(o1)*sin(o3) + cos(o2)*cos(o3)*sin(o1)) - cos(o4)*sin(o1)*sin(o2)))/500 - (63*cos(o6)*(cos(o5)*(cos(o4)*(cos(o1)*sin(o3) + cos(o2)*cos(o3)*sin(o1)) + sin(o1)*sin(o2)*sin(o4)) + sin(o5)*(cos(o1)*cos(o3) - cos(o2)*sin(o1)*sin(o3))))/500;
    J(2,5)=  (63*cos(o6)*(cos(o5)*(cos(o2)*sin(o4) - cos(o3)*cos(o4)*sin(o2)) + sin(o2)*sin(o3)*sin(o5)))/500 - (63*sin(o6)*(cos(o2)*cos(o4) + cos(o3)*sin(o2)*sin(o4)))/500;
    J(3,5)= cos(o5)*(cos(o3)*sin(o1) + cos(o1)*cos(o2)*sin(o3)) - sin(o5)*(cos(o4)*(sin(o1)*sin(o3) - cos(o1)*cos(o2)*cos(o3)) - cos(o1)*sin(o2)*sin(o4));
    J(4,5)=  sin(o5)*(cos(o4)*(cos(o1)*sin(o3) + cos(o2)*cos(o3)*sin(o1)) + sin(o1)*sin(o2)*sin(o4)) - cos(o5)*(cos(o1)*cos(o3) - cos(o2)*sin(o1)*sin(o3));
    J(5,5)= cos(o5)*sin(o2)*sin(o3) - sin(o5)*(cos(o2)*sin(o4) - cos(o3)*cos(o4)*sin(o2));
    J(0,6)= 0;
    J(1,6)= 0;
    J(2,6)= 0;
    J(3,6)= sin(o6)*(cos(o5)*(cos(o4)*(sin(o1)*sin(o3) - cos(o1)*cos(o2)*cos(o3)) - cos(o1)*sin(o2)*sin(o4)) + sin(o5)*(cos(o3)*sin(o1) + cos(o1)*cos(o2)*sin(o3))) - cos(o6)*(sin(o4)*(sin(o1)*sin(o3) - cos(o1)*cos(o2)*cos(o3)) + cos(o1)*cos(o4)*sin(o2));
    J(4,6)=  cos(o6)*(sin(o4)*(cos(o1)*sin(o3) + cos(o2)*cos(o3)*sin(o1)) - cos(o4)*sin(o1)*sin(o2)) - sin(o6)*(cos(o5)*(cos(o4)*(cos(o1)*sin(o3) + cos(o2)*cos(o3)*sin(o1)) + sin(o1)*sin(o2)*sin(o4)) + sin(o5)*(cos(o1)*cos(o3) - cos(o2)*sin(o1)*sin(o3)));
    J(5,6)= sin(o6)*(cos(o5)*(cos(o2)*sin(o4) - cos(o3)*cos(o4)*sin(o2)) + sin(o2)*sin(o3)*sin(o5)) + cos(o6)*(cos(o2)*cos(o4) + cos(o3)*sin(o2)*sin(o4)) ;

J1=J;

std::cout << J1 << std::endl;
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



//add the pinv of Eigen
template<typename _Matrix_Type_> _Matrix_Type_ pseudoInverse(const _Matrix_Type_ &a, double epsilon=std::numeric_limits<double>::epsilon()) 
{  
    Eigen::JacobiSVD< _Matrix_Type_ > svd(a ,Eigen::ComputeThinU | Eigen::ComputeThinV);  
    double tolerance = epsilon * std::max(a.cols(), a.rows()) *svd.singularValues().array().abs()(0);  
    return svd.matrixV() *  (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(), 0).matrix().asDiagonal() * svd.matrixU().adjoint(); 
}   



int main(int argc, char **argv)
{
  ros::init(argc, argv, "listenerRobot");

  ros::NodeHandle n;

  ros::Subscriber sub2=n.subscribe("/iiwa/state/CartesianPose", 1000, chatterCallback1);

  ros::Subscriber sub4=n.subscribe("/iiwa/state/JointPosition", 1000, chatterCallback3);

  ros::Publisher chatter_pub3 = n.advertise<iiwa_msgs::JointPosition>("/iiwa/command/JointPosition", 1000);		


  ros::Rate loop_rate(100);

//阻抗初始化
Matrix<float, 3, 3> U1;
Matrix<float, 3, 3> U2;
Matrix<float, 3, 1> xdot;
Matrix<float, 3, 1> xdotprev;
Matrix<float, 7, 1> qdot;
Matrix<int, 3, 1> fq;
Matrix<float, 3, 1> f_2;
float Ts=0.02;//iiwa控制环频率
xdot.fill(0);
xdotprev.fill(0);

Matrix<float,3, 7> JT; 
Matrix<float,3, 7> JR; 
Matrix<float,3, 7> JT_aug;
Matrix<float,7, 7> eye;
eye.setIdentity(7,7);


while (ros::ok())
 {

    iiwa_msgs::JointPosition msg1;

    Jg(O,J1); //用Jg函数求出此时的雅格比

    /*
    JT=J1.topRows(3);//JT=J1.topRows<3>();
    JR=J1.bottomRows(3);
    J_pinvzhijie2(JR,JR_p);
    JT_aug=JT*(eye-JR_p*JR);//(3,7)
    J_pinvzhijie(JT_aug,Jp);//用直接发求伪逆  */

    J_pinv_svd=pseudoInverse(Jg); //用现成Eigen函数求伪逆

    xdot=; //DS模型输入的xd
    qdot=J_pinv_svd*xdot;


    Oe=Ts*qdot+O;
    xdotprev=xdot;

    msg1.position.a1=Oe(0);
    msg1.position.a2=Oe(1);
    msg1.position.a3=Oe(2);
    msg1.position.a4=Oe(3);
    msg1.position.a5=Oe(4);
    msg1.position.a6=Oe(5);
    msg1.position.a7=Oe(6);

  chatter_pub3.publish(msg1);

//   ROS_INFO("%f,%f,%f,%f,%f,%f", Jpf(0,0),Jpf(0,1),Jpf(0,2),Jpf(0,3),Jpf(0,4),Jpf(0,5));
//   ROS_INFO("%f,%f,%f,%f,%f,%f", Jp(0,0),Jp(0,1),Jp(0,2),Jp(0,3),Jp(0,4),Jp(0,5));
//   ROS_INFO("%f,%f,%f,%f,%f,%f", J1(0,0),J1(0,1),J1(0,2),J1(0,3),J1(0,4),J1(0,5));
   ROS_INFO("%f,%f,%f,%f,%f,%f", qdot(0),qdot(1),qdot(2),qdot(3),qdot(4),qdot(5));
 ROS_INFO("%f,%f,%f", xdot(0),xdot(1),xdot(2));

  ros::spinOnce();

  loop_rate.sleep();

 }

  return 0;
}
