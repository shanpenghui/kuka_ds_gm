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
#include <vector>
#include "Eigen/Eigen"
#include <Eigen/Dense>
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


void Jg(Eigen::Matrix<float,7,1> Data, Eigen::Matrix<float,6,7> J)  //Jagebizhuangzhi put t into F       
////void Jg(Eigen::Matrix<double,7,1> Data, Eigen::Matrix<double,6,7> J)
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

//void J_pinv(Eigen::Matrix<float,6,7> Data, Eigen::Matrix<float,7,6> Jpinv)
//{
//    std::vector<std::vector<float>> vec{
//{Data(0,0),Data(0,1),Data(0,2),Data(0,3),Data(0,4),Data(0,5),Data(0,6)},
//{Data(1,0),Data(1,1),Data(1,2),Data(1,3),Data(1,4),Data(1,5),Data(1,6)},
//{Data(2,0),Data(2,1),Data(2,2),Data(2,3),Data(2,4),Data(2,5),Data(2,6)},
//{Data(3,0),Data(3,1),Data(3,2),Data(3,3),Data(3,4),Data(3,5),Data(3,6)},
//{Data(4,0),Data(4,1),Data(4,2),Data(4,3),Data(4,4),Data(4,5),Data(4,6)},
//{Data(5,0),Data(5,1),Data(5,2),Data(5,3),Data(5,4),Data(5,5),Data(5,6)},
// };  
//    const int rows{ 6 }, cols{ 7 };  
//  
//    std::vector<float> vec_;  
//    for (int i = 0; i < rows; ++i) {  
//        vec_.insert(vec_.begin() + i * cols, vec[i].begin(), vec[i].end());  


//    }  
//    Eigen::Map<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> m(vec_.data(), rows, cols);  
//  
////    fprintf(stderr, "source matrix:\n");  
////    std::cout << m << std::endl;  
//  
////    fprintf(stderr, "\nEigen implement pseudoinverse:\n");  
//    auto svd = m.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV);  
//  
//    const auto &singularValues = svd.singularValues();  
//    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> singularValuesInv(m.cols(), m.rows());  
//    singularValuesInv.setZero();  
//    double  pinvtoler = 1.e-6; // choose your tolerance wisely  
//    for (unsigned int i = 0; i < singularValues.size(); ++i) {  
//        if (singularValues(i) > pinvtoler)  
//            singularValuesInv(i, i) = 1.0f / singularValues(i);  
//        else  
//            singularValuesInv(i, i) = 0.f;  
//    }  
//  
//    Eigen::MatrixXf pinvmat = svd.matrixV() * singularValuesInv * svd.matrixU().transpose();  
//    Jpinv=pinvmat;
//    std::cout << Jpinv << std::endl;  
//  
////    return 0;  
//}

//void J_pinv1(Eigen::Matrix<float,7,6> Data, Eigen::Matrix<float,6,7> Jpinv)Cartesianimpedancecontrol
//{
//    std::vector<std::vector<float>> vec{
//{Data(0,0),Data(0,1),Data(0,2),Data(0,3),Data(0,4),Data(0,5)},
//{Data(1,0),Data(1,1),Data(1,2),Data(1,3),Data(1,4),Data(1,5)},
//{Data(2,0),Data(2,1),Data(2,2),Data(2,3),Data(2,4),Data(2,5)},
//{Data(3,0),Data(3,1),Data(3,2),Data(3,3),Data(3,4),Data(3,5)},
//{Data(4,0),Data(4,1),Data(4,2),Data(4,3),Data(4,4),Data(4,5)},
//{Data(5,0),Data(5,1),Data(5,2),Data(5,3),Data(5,4),Data(5,5)},
//{Data(6,0),Data(6,1),Data(6,2),Data(6,3),Data(6,4),Data(6,5)},
// };  
//    const int rows{ 7 }, cols{ 6 };  
//  
//    std::vector<float> vec_;  
//    for (int i = 0; i < rows; ++i) {  
//        vec_.insert(vec_.begin() + i * cols, vec[i].begin(), vec[i].end());  
//    }  
//    Eigen::Map<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> m(vec_.data(), rows, cols);  
//  
////    fprintf(stderr, "source matrix:\n");  
////    std::cout << m << std::endl;  
//  
////    fprintf(stderr, "\nEigen implement pseudoinverse:\n");  
//    auto svd = m.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV);  
//  
//    const auto &singularValues = svd.singularValues();  
//    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> singularValuesInv(m.cols(), m.rows());  
//    singularValuesInv.setZero();  
//    double  pinvtoler = 1.e-6; // choose your tolerance wisely  
//    for (unsigned int i = 0; i < singularValues.size(); ++i) {  
//        if (singularValues(i) > pinvtoler)  
//            singularValuesInv(i, i) = 1.0f / singularValues(i);  
//        else  
//            singularValuesInv(i, i) = 0.f;  
//    }  
//  
//    Eigen::MatrixXf pinvmat = svd.matrixV() * singularValuesInv * svd.matrixU().transpose();  
//    Jpinv=pinvmat;
//    std::cout << Jpinv << std::endl;  
//  
////    return 0;  
//}


void J_pinvzhijie(Eigen::Matrix<float,Dynamic, Dynamic> Data, Eigen::Matrix<float,Dynamic, Dynamic> Jpinv)
{//用公式pinv(J)=J'*inv(J*J'+lambda*eye(size(H))

Matrix<float,Dynamic, Dynamic> J6;//中间变量
Matrix<float,Dynamic, Dynamic> J7;
Matrix<float,Dynamic, Dynamic> J8;
Matrix<float,Dynamic, Dynamic> J9;

J6=Data.transpose();//对要求伪逆的矩阵进行转置
J9.setIdentity(Data.rows(),Data.rows());
J7=Data*J6+0.001*J9;//J*J'+lambda*eye(size(H)
J8=J7.inverse();//inv(J*J'+lambda*eye(size(H))
Jpinv=J6*J8;
Jp=Jpinv;
//ROS_INFO("%f,%f,%f,%f,%f,%f", Data(0,0),Data(0,1),Data(0,2),Data(0,3),Data(0,4),Data(0,5));
//ROS_INFO("%f,%f,%f,%f,%f,%f", J6(0,0),J6(0,1),J6(0,2),J6(0,3),J6(0,4),J6(0,5));
//ROS_INFO("%f,%f,%f,%f,%f,%f", J7(0,0),J7(0,1),J7(0,2),J7(0,3),J7(0,4),J7(0,5));
//ROS_INFO("%f,%f,%f,%f,%f,%f", J8(0,0),J8(0,1),J8(0,2),J8(0,3),J8(0,4),J8(0,5));
//ROS_INFO("%f,%f,%f,%f,%f,%f", J9(0,0),J9(0,1),J9(0,2),J9(0,3),J9(0,4),J9(0,5));
}

void J_pinvzhijie1(Eigen::Matrix<float,Dynamic, Dynamic> Data, Eigen::Matrix<float,Dynamic, Dynamic> Jpinv)
{//用公式pinv(J)=J'*inv(J*J'+lambda*eye(size(H))

Matrix<float,Dynamic, Dynamic> J6;//中间变量
Matrix<float,Dynamic, Dynamic> J7;
Matrix<float,Dynamic, Dynamic> J8;
Matrix<float,Dynamic, Dynamic> J9;

J6=Data.transpose();//对要求伪逆的矩阵进行转置
J9.setIdentity(Data.rows(),Data.rows());
J7=Data*J6+0.001*J9;//J*J'+lambda*eye(size(H)
J8=J7.inverse();//inv(J*J'+lambda*eye(size(H))
Jpinv=J6*J8;
Jpf=Jpinv;
//ROS_INFO("%f,%f,%f,%f,%f,%f", Data(0,0),Data(0,1),Data(0,2),Data(0,3),Data(0,4),Data(0,5));
//ROS_INFO("%f,%f,%f,%f,%f,%f", J6(0,0),J6(0,1),J6(0,2),J6(0,3),J6(0,4),J6(0,5));
//ROS_INFO("%f,%f,%f,%f,%f,%f", J7(0,0),J7(0,1),J7(0,2),J7(0,3),J7(0,4),J7(0,5));
//ROS_INFO("%f,%f,%f,%f,%f,%f", J8(0,0),J8(0,1),J8(0,2),J8(0,3),J8(0,4),J8(0,5));
//ROS_INFO("%f,%f,%f,%f,%f,%f", J9(0,0),J9(0,1),J9(0,2),J9(0,3),J9(0,4),J9(0,5));
}

void J_pinvzhijie2(Eigen::Matrix<float,Dynamic, Dynamic> Data, Eigen::Matrix<float,Dynamic, Dynamic> Jpinv)
{//用公式pinv(J)=J'*inv(J*J'+lambda*eye(size(H))

Matrix<float,Dynamic, Dynamic> J6;//中间变量
Matrix<float,Dynamic, Dynamic> J7;
Matrix<float,Dynamic, Dynamic> J8;
Matrix<float,Dynamic, Dynamic> J9;

J6=Data.transpose();//对要求伪逆的矩阵进行转置
J9.setIdentity(Data.rows(),Data.rows());
J7=Data*J6+0.001*J9;//J*J'+lambda*eye(size(H)
J8=J7.inverse();//inv(J*J'+lambda*eye(size(H))
Jpinv=J6*J8;
JR_p=Jpinv;
//ROS_INFO("%f,%f,%f,%f,%f,%f", Data(0,0),Data(0,1),Data(0,2),Data(0,3),Data(0,4),Data(0,5));
//ROS_INFO("%f,%f,%f,%f,%f,%f", J6(0,0),J6(0,1),J6(0,2),J6(0,3),J6(0,4),J6(0,5));
//ROS_INFO("%f,%f,%f,%f,%f,%f", J7(0,0),J7(0,1),J7(0,2),J7(0,3),J7(0,4),J7(0,5));
//ROS_INFO("%f,%f,%f,%f,%f,%f", J8(0,0),J8(0,1),J8(0,2),J8(0,3),J8(0,4),J8(0,5));
//ROS_INFO("%f,%f,%f,%f,%f,%f", J9(0,0),J9(0,1),J9(0,2),J9(0,3),J9(0,4),J9(0,5));
}

//////////////////////回调函数////////

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

  ros::Subscriber sub1=n.subscribe("adc", 1000, chatterCallback);

  ros::Subscriber sub2=n.subscribe("/iiwa/state/CartesianPose", 1000, chatterCallback1);

  ros::Subscriber sub3=n.subscribe("/iiwa/state/JointTorque", 1000, chatterCallback2);

  ros::Subscriber sub4=n.subscribe("/iiwa/state/JointPosition", 1000, chatterCallback3);

 // ros::Subscriber sub5=n.subscribe("/iiwa/state/CartesianWrench", 1000, chatterCallback4);

ros::Subscriber sub5=n.subscribe("/ft_sensor/data", 1000, chatterCallback4);

  ros::Publisher chatter_pub1 = n.advertise<std_msgs::Int16>("EMGs", 1000);

//  ros::Publisher chatter_pub2 = n.advertise<geometry_msgs::PoseStamped>("/iiwa/command/CartesianPose", 1000);

  ros::Publisher chatter_pub3 = n.advertise<iiwa_msgs::JointPosition>("/iiwa/command/JointPosition", 1000);

//  ros::Publisher chatter_pub4 = n.advertise<iiwa_msgs::JointVelocity>("/iiwa/command/JointVelocity", 1000);

  ros::Rate loop_rate(200);

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
Cd=MatrixXf::Zero(3,3); 
Md=MatrixXf::Zero(3,3); 
Matrix<float,3, 3> Cdd; 
Matrix<float,3, 3> Mdd; 


Cdd.setIdentity(3,3);
Cd=10*Cdd;

Mdd.setIdentity(3,3);
Md=0.5*Cdd;

Matrix<float,3, 7> JT; 
Matrix<float,3, 7> JR; 
Matrix<float,3, 7> JT_aug;
Matrix<float,7, 7> eye;
eye.setIdentity(7,7);

am=1;//肌肉最大主动力
bm=1;

while (ros::ok())
 {

//////////////////////////////////2肌电信号处理////////////////////////////

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



//a1=c/950;//归一化
//b1=d/950;

//ha1=160.0/950.0;//阈值设计

//  if(a1>b1){//计算腕关节共收缩程度

//    cw=a1;
//    if(cw>ha1){//计算腕关节共收缩程度 
//      s=1000;
//       }
//    else{
//    s=20;
//       }
//   }
//   else{
//    cw=b1;
//    if(cw>ha1) {//计算腕关节共收缩程度
//    s=1000;
//     }
//    else{
//    s=20;
//     }
//   }



//  msg.data = s;

////  ROS_INFO("%d", msg.data);
// ROS_INFO("%f,%f", cw,ha1);
////ROS_INFO("%f,%f", a1,b1);
////ROS_INFO("%d,%d", c,d);
//chatter_pub1.publish(msg);
////////////////////////////////////////////////////


////////////////////////////////关节导纳控制器(三方向)//////////////////////////////////

    iiwa_msgs::JointPosition msg1;

    Jg(O,J1); //用Jg函数求出此时的雅格比
    JT=J1.topRows(3);//JT=J1.topRows<3>();
    JR=J1.bottomRows(3);
    J_pinvzhijie2(JR,JR_p);
    JT_aug=JT*(eye-JR_p*JR);//(3,7)
    J_pinvzhijie(JT_aug,Jp);//用直接发求伪逆

  for (int i = 0; i < 3; ++i) //消去力的震动
{   
    W(i)=W(i)*10;
    fq(i)=W(i);
    f_2(i)=fq(i);
    f_2(i)=f_2(i)/10;
}

    U1=Md/Ts + Cd;
    U2=U1.inverse();
    xdot=U2*(Md*xdotprev/Ts+f_2);  //如果只求到这一步，输出的位置是很稳的
//    xdot(2)=0;
    qdot=Jp*xdot;


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
//ROS_INFO("%f,%f,%f", f_2(0),f_2(1),f_2(2));


//    int icount, jcount;

//    printf("The Matrix J1 is:\n");
//    for (icount=0;icount<6;icount++)
//    {
//        for (jcount=0;jcount<7;jcount++)
//        {
//            printf("%f ",J1(icount,jcount));
//        }
//        printf("\n");
//    }  

//    printf("The Matrix Jp is:\n");
//    for (icount=0;icount<7;icount++)
//    {
//        for (jcount=0;jcount<3;jcount++)
//        {
//            printf("%f ",Jp(icount,jcount));
//        }
//        printf("\n");
//    }  

/////////////////////////////////////////////////////////////////

  ros::spinOnce();

  loop_rate.sleep();

 }

  return 0;
}
