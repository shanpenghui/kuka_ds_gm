#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float32.h"
#include "rosserial_arduino/Adc.h"
#include "geometry_msgs/PoseStamped.h"
#include <iostream>  
#include "math.h"  
#include <stdio.h>
#include <sstream>
#include <vector>
#include "Eigen/Eigen"
#include <Eigen/Dense>
#include <string>
// #include <Eigen/Core> 
// #include <Eigen/SVD>   
// #include <Eigen/QR>  

#include "iiwaRos.h"//clint的头文件
#include <cstdlib>
#include <iiwa_msgs/ConfigureSmartServo.h>
#include <iiwa_msgs/SmartServoMode.h>
#include <sensor_msgs/JointState.h>
#define PI 3.1415926


using namespace std;
using namespace Eigen;


//////////////////////定义变量/////////////////
Matrix<float, 7, 6> J2;//雅格比转置
Matrix<float, 6, 7> Jpf;//雅格比转置后的伪逆
Matrix<float, 7, 1> O;//关节量
Matrix<float, 7, 1> Oe;
Matrix<float, 6, 1> W;//笛卡尔力
Matrix<float, 7, 1>  T;//关节力矩向量
Matrix<float, 7, 1>  R;//笛卡尔位置向量
Matrix<float, 7, 1>  Re;//笛卡尔位置向量
Matrix<float, 3, 1>  V;
Matrix<float,7, 3> JR_p;
Matrix<float, 7, 3> Jp;//雅格比伪逆
Eigen::Matrix<float,3,1> pose_in_callback;//for void forwardkinemic
Eigen::Matrix<float,6,7> J1;//for void Jg
//阻抗初始化
Matrix<float, 3, 1> xdot;
Matrix<float, 7, 1> qdot;
float Ts=0.0039;//iiwa控制环频率
Matrix<float,3, 7> JT; 
Matrix<float,3, 7> JR; 
Matrix<float,3, 7> JT_aug;
Matrix<float,7, 7> eye;





// //网上的iiwa你运动学结算
// double kukaJoints[7];
// // DH Parameters for the KUKA. Refer to Descriptions.png
// double D1 = 0.36;
// double D3 = 0.42;
// double D5 = 0.40;
// double D7 = 0.126;
// double JOINT_LIMITS[] = { 170.0*PI / 180.0,
// 120.0*PI / 180.0,
// 170.0*PI / 180.0,
// 120.0*PI / 180.0,
// 170.0*PI / 180.0,
// 120.0*PI / 180.0,
// 175.0*PI / 180.0 };
// /*----------------------------------------------------------------------------*/
// bool inverseKuka(double poseDsr[])
// {
// 	//    // Returning value
// 	bool validIK = true;
// 	// Initialize variables
// 	//    	// Auxiliary variables
// 	double mod_pW, mod_pWxy, c2, s2, c3, s3;
// 	//    		// Getting transformation matrix
// 	Eigen::Quaterniond R = Eigen::Quaterniond(poseDsr[3], poseDsr[4], poseDsr[5], poseDsr[6]);
// 	//    			//	The -ve sign for the desired pos. is due to the difference between V-REP and code
// 	Eigen::Vector3d p(-poseDsr[0], -poseDsr[1], poseDsr[2] - D1);
// 	//    			// Inverse kinematics calculations. Closed form solution.
// 	//    			// hand position
// 	Eigen::Vector3d pW = p - D7 * R.toRotationMatrix().col(2);
// 	// Calculate wrist position
// 	kukaJoints[0] = atan2(pW[1], pW[0]);
// 	mod_pW = pow(pW.norm(), 2); // Pwx^2+Pwy^2+Pwz^2
// 	//
// 	c3 = (mod_pW - D3*D3 - D5*D5) / (2 * D3*D5);
// 	// If c3>1, there is no solution for IKT
// 	if (c3>1){
// 		validIK = false;
// 		printf("ikSolver: Attempt to access to a point out of the workspace. Zero array will be returned.\n");
// 		for (int j = 0; j<7; j++)kukaJoints[j] = 0;
// 		return validIK;
// 	}
// 	//
// 	s3 = -sqrt(1 - c3*c3);
// 	kukaJoints[3] = atan2(s3, c3) + PI / 2;
// 	//
// 	//    					// We do not use the extra dof for getting the inverse kinematics
// 	kukaJoints[2] = 0.0;
// 	//
// 	mod_pWxy = sqrt(pW[0] * pW[0] + pW[1] * pW[1]);
// 	s2 = ((D3 + D5*c3)*pW[2] - D5*s3*mod_pWxy) / mod_pW;
// 	c2 = ((D3 + D5*c3)*mod_pWxy + D5*s3*pW[2]) / mod_pW;
// 	kukaJoints[1] = atan2(s2, c2);
// 	//
// 	//    					// Calculate orientation (angles of the wrist joints)
// 	Eigen::Matrix3d T01; T01 << cos(kukaJoints[0]), 0.0, sin(kukaJoints[0]), sin(kukaJoints[0]), 0.0, -cos(kukaJoints[0]), 0.0, 1.0, 0.0;
// 	Eigen::Matrix3d T12; T12 << cos(kukaJoints[1]), -sin(kukaJoints[1]), 0.0, sin(kukaJoints[1]), cos(kukaJoints[1]), 0.0, 0.0, 0.0, 1.0;
// 	Eigen::Matrix3d T23; T23 << cos(kukaJoints[3]), 0.0, sin(kukaJoints[3]), sin(kukaJoints[3]), 0.0, -cos(kukaJoints[3]), 0.0, 1.0, 0.0;
// 	//
// 	Eigen::Matrix3d pose03 = T01*T12*T23;
// 	Eigen::Matrix3d  pose36 = pose03.inverse() * (R.toRotationMatrix());
// 	//
// 	kukaJoints[4] = atan2(pose36(1, 2), pose36(0, 2));
// 	kukaJoints[5] = atan2(sqrt(pose36(0, 2)*pose36(0, 2) + pose36(1, 2)*pose36(1, 2)), pose36(2, 2));
// 	kukaJoints[6] = atan2(pose36(2, 1), -pose36(2, 0));
// 	//
// 	//    					//Adjust to robot from IK coordinates (keeping joint coord. within the interval [-pi,pi])
// 	kukaJoints[1] < -PI / 2 ? kukaJoints[1] += 3 * PI / 2 : kukaJoints[1] -= PI / 2;
// 	kukaJoints[3] < -PI / 2 ? kukaJoints[3] += 3 * PI / 2 : kukaJoints[3] -= PI / 2;
// 	kukaJoints[6] <     0 ? kukaJoints[6] += PI : kukaJoints[6] -= PI;
// 	//
// 	kukaJoints[3] = -kukaJoints[3]; //Correcting for the RobotRotation
// 	//
// 	for (int i = 0; i < 7; i++){
// 		if (abs(kukaJoints[i]) > JOINT_LIMITS[i]){
// 			validIK = false;
// 			kukaJoints[i] > 0 ? kukaJoints[i] = JOINT_LIMITS[i] : kukaJoints[i] = -JOINT_LIMITS[i];
// 			printf("Warning!!! IK gives values out of bounds for joint %d \n", i);
// 		}
// 		//
// 		printf("Joint [%d]: %.3f \n", i, kukaJoints[i] * 180 / PI);
// 	}
// 	//
// 	//
// 	return validIK;
// 	//
// }
// /////////////////////////////////////////////////////

void Jg(const Eigen::Matrix<float,7,1> Data)  
{  
    float o1 = 0; float o2 = 0;float o3 = 0;float o4 = 0;float o5 = 0;float o6 = 0;float o7 = 0;
    o1=Data[0];o2=Data[1];o3=Data[2];o4=Data[3];o5=Data[4];o6=Data[5];o7=Data[6];
    Eigen::Matrix<float,6,7> J;
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

//  std::cout << J1 << std::endl;
//ROS_INFO("%f,%f,%f,%f,%f,%f,%f", o1,o2,o3,o4,o5,o6,o7);
}

void forwardkinemic(const Eigen::Matrix<float,7,1> Data)
{
    float o1 = 0; float o2 = 0;float o3 = 0;float o4 = 0;float o5 = 0;float o6 = 0;float o7 = 0;
    float xp;float yp;float zp;
          o1=Data[0];o2=Data[1];o3=Data[2];o4=Data[3];o5=Data[4];o6=Data[5];o7=Data[6];

          xp = (63*sin(o6)*(cos(o5)*(cos(o4)*(sin(o1)*sin(o3) - cos(o1)*cos(o2)*cos(o3)) - cos(o1)*sin(o2)*sin(o4)) + sin(o5)*(cos(o3)*sin(o1) + cos(o1)*cos(o2)*sin(o3))))/500 - (63*cos(o6)*(sin(o4)*(sin(o1)*sin(o3) - cos(o1)*cos(o2)*cos(o3)) + cos(o1)*cos(o4)*sin(o2)))/500 - (21*cos(o1)*sin(o2))/50 - (2*sin(o4)*(sin(o1)*sin(o3) - cos(o1)*cos(o2)*cos(o3)))/5 - (2*cos(o1)*cos(o4)*sin(o2))/5;//- (21*cos(o1)*sin(o2))/50 - (2*sin(o4)*(sin(o1)*sin(o3) - cos(o1)*cos(o2)*cos(o3)))/5 - (2*cos(o1)*cos(o4)*sin(o2))/5;
          yp = (63*cos(o6)*(sin(o4)*(cos(o1)*sin(o3) + cos(o2)*cos(o3)*sin(o1)) - cos(o4)*sin(o1)*sin(o2)))/500 - (21*sin(o1)*sin(o2))/50 - (63*sin(o6)*(cos(o5)*(cos(o4)*(cos(o1)*sin(o3) + cos(o2)*cos(o3)*sin(o1)) + sin(o1)*sin(o2)*sin(o4)) + sin(o5)*(cos(o1)*cos(o3) - cos(o2)*sin(o1)*sin(o3))))/500 + (2*sin(o4)*(cos(o1)*sin(o3) + cos(o2)*cos(o3)*sin(o1)))/5 - (2*cos(o4)*sin(o1)*sin(o2))/5;//(2*sin(o4)*(cos(o1)*sin(o3) + cos(o2)*cos(o3)*sin(o1)))/5 - (21*sin(o1)*sin(o2))/50 - (2*cos(o4)*sin(o1)*sin(o2))/5;
          zp = (21*cos(o2))/50 + (2*cos(o2)*cos(o4))/5 + (63*sin(o6)*(cos(o5)*(cos(o2)*sin(o4) - cos(o3)*cos(o4)*sin(o2)) + sin(o2)*sin(o3)*sin(o5)))/500 + (63*cos(o6)*(cos(o2)*cos(o4) + cos(o3)*sin(o2)*sin(o4)))/500 + (2*cos(o3)*sin(o2)*sin(o4))/5 +0.36;//+ 9/25   (21*cos(o2))/50 + (2*cos(o2)*cos(o4))/5 + (2*cos(o3)*sin(o2)*sin(o4))/5;

            pose_in_callback[0]=-xp;
            pose_in_callback[1]=-yp;
            pose_in_callback[2]=zp;

        //   pose_in_callback.pose.position.x=xp;
        //   pose_in_callback.pose.position.y=yp;
        //   pose_in_callback.pose.position.z=zp;
        //   pose_in_callback.pose.orientation.x=0;
        //   pose_in_callback.pose.orientation.y=0;
        //   pose_in_callback.pose.orientation.z=0;
        //   pose_in_callback.pose.orientation.w=0;

        //   ROS_INFO("%f,%f,%f", pose.pose.position.x,pose.pose.position.y,pose.pose.position.z);
           //ROS_INFO("%f,%f,%f", xp,yp,zp);
            //ROS_INFO("%f,%f,%f", pose_in_callback[0],pose_in_callback[1],pose_in_callback[2]);
          //ROS_INFO("%f,%f,%f,%f,%f,%f,%f", o1,o2,o3,o4,o5,o6,o7);
          //ROS_INFO("%f,%f,%f,%f,%f,%f,%f", Data[0],Data[1],Data[2],Data[3],Data[4],Data[5],Data[6],);
}


void chatterCallback1(const geometry_msgs::PoseStamped& msg)  //put CartesianPose into r
{
float r1 = 0;float r2 = 0;float r3 = 0;float r4 = 0;float r5 = 0;float r6 = 0;float r7 = 0;
// ROS_INFO("%f,%f,%f,%f,%f,%f,%f", msg.pose.position.x,msg.pose.position.y,msg.pose.position.z,msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w);
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

void chatterCallback2(const geometry_msgs::PoseStamped& msg)  //put CartesianPose into r
{
float v1 = 0;float v2 = 0;float v3 = 0;
//
    v1 = msg.pose.position.x;
    v2 = msg.pose.position.y;
    v3 = msg.pose.position.z;

    V[0]=-v1;
    V[1]=-v2;
    V[2]=v3;
    //OS_INFO("%f,%f,%f", V[0],V[1],V[2]);
}


void chatterCallback3(const sensor_msgs::JointState& msg) //sensor_msgs::JointState   sensor_msgs::JointStateConstPtr
{
float o1 = 0; float o2 = 0;float o3 = 0;float o4 = 0;float o5 = 0;float o6 = 0;float o7 = 0;
    o1 = msg.position[0];
    o2 = msg.position[1];
    o3 = msg.position[2];
    o4 = msg.position[3];
    o5 = msg.position[4];
    o6 = msg.position[5];
    o7 = msg.position[6];   

    O[0]=o1;
    O[1]=o2;
    O[2]=o3;
    O[3]=o4;
    O[4]=o5;
    O[5]=o6;
    O[6]=o7;
//ROS_INFO("%f,%f,%f,%f,%f,%f,%f", msg.position[0], msg.position[1], msg.position[2], msg.position[3], msg.position[4], msg.position[5], msg.position[6]);
 
ROS_INFO("%f,%f,%f,%f,%f,%f,%f", o1,o2,o3,o4,o5,o6,o7);
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
// std::cout << JR_p << std::endl;
}

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
//std::cout << Jp << std::endl;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "joint_position_for_gazebo");

  ros::NodeHandle n;

  ros::Subscriber sub1=n.subscribe("/iiwa/command/CartesianPose", 1000, chatterCallback1);

  ros::Subscriber sub2=n.subscribe("/iiwa/command/CartesianVelocity", 1000, chatterCallback2);

  ros::Subscriber sub3=n.subscribe("/iiwa/joint_states", 1000, chatterCallback3);

  ros::Publisher chatter_pub0 = n.advertise<geometry_msgs::PoseStamped>("/iiwa/state/CartesianPose", 1000);	

  ros::Publisher chatter_pub1 = n.advertise<std_msgs::Float64>("/iiwa/PositionJointInterface_J1_controller/command", 1000);	
  ros::Publisher chatter_pub2 = n.advertise<std_msgs::Float64>("/iiwa/PositionJointInterface_J2_controller/command", 1000);	
  ros::Publisher chatter_pub3 = n.advertise<std_msgs::Float64>("/iiwa/PositionJointInterface_J3_controller/command", 1000);	
  ros::Publisher chatter_pub4 = n.advertise<std_msgs::Float64>("/iiwa/PositionJointInterface_J4_controller/command", 1000);	
  ros::Publisher chatter_pub5 = n.advertise<std_msgs::Float64>("/iiwa/PositionJointInterface_J5_controller/command", 1000);	
  ros::Publisher chatter_pub6 = n.advertise<std_msgs::Float64>("/iiwa/PositionJointInterface_J6_controller/command", 1000);	
  ros::Publisher chatter_pub7 = n.advertise<std_msgs::Float64>("/iiwa/PositionJointInterface_J7_controller/command", 1000);		

  ros::Rate loop_rate(256);




while (ros::ok())
 {

    std_msgs::Float64 msg1;
    std_msgs::Float64 msg2;
    std_msgs::Float64 msg3;
    std_msgs::Float64 msg4;
    std_msgs::Float64 msg5;
    std_msgs::Float64 msg6;
    std_msgs::Float64 msg7;

    Eigen::Matrix<float,3,1> posexyz;
    geometry_msgs::PoseStamped pose_2_pub;
    
xdot.fill(0);
qdot.fill(0);
Oe.fill(0);
eye.setIdentity(7,7);
///////////////////////////////////////自己的雅格比逆运动学
    Jg(O); //用Jg函数求出此时的雅格比
    forwardkinemic(O);

    JT=J1.topRows(3);//JT=J1.topRows<3>();
    JR=J1.bottomRows(3);
    J_pinvzhijie2(JR,JR_p);
    JT_aug=JT*(eye-JR_p*JR);//(3,7)
    J_pinvzhijie(JT_aug,Jp);//用直接发求伪逆  

    // ROS_INFO("%f,%f,%f,%f,%f,%f,%f", o1,o2,o3,o4,o5,o6,o7);
    // std::cout << JR_p << std::endl;
    /*
    Eigen::MatrixXd A = J1;
    Eigen::MatrixXd J_pinv_svd = A.completeOrthogonalDecomposition().pseudoInverse();//用现成Eigen函数求伪逆
    */

    xdot=V; //DS模型输入的xd
    qdot=Jp*xdot;
    Oe=Ts*qdot+O;

    msg1.data=Oe(0);
    msg2.data=Oe(1);
    msg3.data=Oe(2);
    msg4.data=Oe(3);
    msg5.data=Oe(4);
    msg6.data=Oe(5);
    msg7.data=Oe(6);
///////////////////////////////////////////////////////////////////////////

//////////////////////////////////别人的逆运动学
    // double Rd[7];
    // Rd[0]=R(0);Rd[1]=R(1);Rd[2]=R(2);Rd[3]=R(3);Rd[4]=R(4);Rd[5]=R(5);Rd[6]=R(6);
    // inverseKuka(Rd);
    // msg1.data=kukaJoints[0];
    // msg2.data=kukaJoints[1];
    // msg3.data=kukaJoints[2];
    // msg4.data=kukaJoints[3];
    // msg5.data=kukaJoints[4];
    // msg6.data=kukaJoints[5];
    // msg7.data=kukaJoints[6];
///////////////////////////////////////////////////////////////////////


 //////////////////////自己写正运动学
    pose_2_pub.pose.position.x=pose_in_callback[0];
    pose_2_pub.pose.position.y=pose_in_callback[1];
    pose_2_pub.pose.position.z=pose_in_callback[2];
    pose_2_pub.pose.orientation.x=0.3844;
    pose_2_pub.pose.orientation.y=0.9232;
    pose_2_pub.pose.orientation.z=0.0014;
    pose_2_pub.pose.orientation.w=0.000035774;
//////////////////////////////////


  chatter_pub0.publish(pose_2_pub);

  chatter_pub1.publish(msg1);
  chatter_pub2.publish(msg2);
  chatter_pub3.publish(msg3);
  chatter_pub4.publish(msg4);
  chatter_pub5.publish(msg5);
  chatter_pub6.publish(msg6);
  chatter_pub7.publish(msg7);

  


//ROS_INFO("%f,%f,%f,%f,%f,%f", qdot(0),qdot(1),qdot(2),qdot(3),qdot(4),qdot(5));
 // ROS_INFO("%f,%f,%f", xdot(0),xdot(1),xdot(2));
// ROS_INFO("%f,%f,%f", x1,y1,z1);
//ROS_INFO("%f,%f,%f", posexyz[0],posexyz[1],posexyz[2]);
// ROS_INFO("%f,%f,%f", pose_in_callback[0],pose_in_callback[1],pose_in_callback[2]);
//ROS_INFO("%f,%f,%f,%f,%f,%f,%f", msg1.data,msg2.data,msg3.data,msg4.data,msg5.data,msg6.data,msg7.data);

  ros::spinOnce();

  loop_rate.sleep();

 }

  return 0;
}
