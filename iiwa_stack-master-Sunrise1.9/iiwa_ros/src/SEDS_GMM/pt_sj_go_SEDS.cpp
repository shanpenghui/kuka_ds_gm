#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <iostream>
#include <vector>
#include "iiwaRos.h"
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/Float64.h"
#include <fstream>
#include <string>
#include <stdio.h>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>
#include <Eigen/Core> 
#include <Eigen/SVD>  
#include "math.h"  
#include "MathLib.h"
#include "GMR.h"
/*The SEDS model that you will use for motion generation.
This model should be accessible in your entire code. */

using namespace std;
using namespace Eigen;
using namespace MathLib;

GaussianMixture mySEDS;
GaussianMixture mySEDS_second;

float kx,ky,kz;
int pt_mode;
geometry_msgs::PoseStamped msg;
geometry_msgs::PoseStamped pose_comd;
geometry_msgs::PoseStamped pose_go;
geometry_msgs::WrenchStamped force_go;
std_msgs::Float64 pt_wrench_x;
std_msgs::Float64 pt_wrench_y;
std_msgs::Float64 pt_wrench_z;
void set_impedance(iiwa_msgs::ConfigureSmartServo& config1,int& pt);
void pose_chatterCallback(const geometry_msgs::PoseStamped& msg1)
{
  msg.pose.position.x=msg1.pose.position.x;
  msg.pose.position.y=msg1.pose.position.y;
  msg.pose.position.z=msg1.pose.position.z;
  msg.pose.orientation.x=msg1.pose.orientation.x;
  msg.pose.orientation.y=msg1.pose.orientation.y;
  msg.pose.orientation.z=msg1.pose.orientation.z;
  msg.pose.orientation.w=msg1.pose.orientation.w;
}
void pose_iiwa_Callback(const geometry_msgs::PoseStamped& msg1)
{
  pose_go.pose.position.x=msg1.pose.position.x;
  pose_go.pose.position.y=msg1.pose.position.y;
  pose_go.pose.position.z=msg1.pose.position.z;
  pose_go.pose.orientation.x =msg1.pose.orientation.x;
  pose_go.pose.orientation.y =msg1.pose.orientation.y;
  pose_go.pose.orientation.z =msg1.pose.orientation.z;
  pose_go.pose.orientation.w =msg1.pose.orientation.w;
  // ROS_INFO("%f", msg1.pose.position.x);
}
void wrench_iiwa_callback(const geometry_msgs::WrenchStamped& msg1)
{
  force_go.wrench.force.x=msg1.wrench.force.x;
  force_go.wrench.force.y=msg1.wrench.force.y;
  force_go.wrench.force.z=msg1.wrench.force.z;
  force_go.wrench.torque.x=msg1.wrench.torque.x;
  force_go.wrench.torque.y=msg1.wrench.torque.y;
  force_go.wrench.torque.z=msg1.wrench.torque.z;
  
//ROS_INFO("listener_wrench:%f,%f,%f", msg1.wrench.force.x,msg1.wrench.force.y,msg1.wrench.force.z);
}

int main(int argc, char **argv)
{
//ROS初始化
  ros::init(argc, argv, "pt_sj_go_SEDS");
  ros::NodeHandle n;
  ros::Subscriber pose_sub = n.subscribe("/pose", 100, pose_chatterCallback);//要加一个运行包，不断发送姿态命令，这样就不用自己编姿态了
  ros::Publisher pose_command_pub = n.advertise<geometry_msgs::PoseStamped>("/iiwa/command/CartesianPose", 1000);
  ros::ServiceClient client = n.serviceClient<iiwa_msgs::ConfigureSmartServo>("/iiwa/configuration/configureSmartServo");
  ros::Subscriber pose_iiwa_sub = n.subscribe("/iiwa/state/CartesianPose", 1000, pose_iiwa_Callback);
  ros::Subscriber wrench_iiwa_sub = n.subscribe("/iiwa/state/CartesianWrench", 1000, wrench_iiwa_callback);
  ros::Publisher pose_go_pub = n.advertise<geometry_msgs::PoseStamped>("pose_go", 1000);
  ros::Publisher force_go_pub = n.advertise<geometry_msgs::WrenchStamped>("force_go", 1000);
  iiwa_msgs::ConfigureSmartServo config;
  ros::Rate loop_rate(256);
  pt_mode=3;

//模型初始化
  //读取第一个seds文件的起始和结尾
    bool b_SEDSLoaded = mySEDS.loadParams("WRSEDSModel1.txt");//WRSEDSModel3.txt
      if (b_SEDSLoaded)
      std::cout << "The SEDS Model is loaded successfully" << std::endl;
      else
      std::cout << "Error: Cannot find the SEDS model!!!" << std::endl;

  //读取第二个seds文件的起始和结尾
    bool b_SEDSLoaded_second = mySEDS_second.loadParams("WRSEDSModel3.txt");//WRSEDSModel3.txt
      if (b_SEDSLoaded_second)
      std::cout << "The SEDS Model TWO is loaded successfully" << std::endl;
      else
      std::cout << "Error: Cannot find the SEDS TWO model!!!" << std::endl;

  //读取GMM文件的起始和结尾以及中间数组
    char filename1[]="WRGMMGMRModel2.txt";

      std::ifstream in(filename1);//, ios::trunc| ios::in | ios::out | ios::binary
      //string line;
      // int line;
    
      if(!in.is_open())
      {
        std::cout<<"GMM file open fail"<<std::endl;
      }
      else
      std::cout<<"GMM The SEDS Model is loaded successfully"<<std::endl;

  //从txt传数据到数组向量
    std::vector<float> x_begin; std::vector<float> x_end; 
    std::array<float,3> x_begin_array; std::array<float,3> x_end_array;
    float x_begin_shuzu[3]; float x_end_shuzu[3];
    std::vector< std::vector<float> > x_position(3, std::vector<float>(0)); 
    float current_number = 0;
    for (int i = 3; i < 6; i++)
      {
        in >> current_number;
        x_begin.push_back(current_number);
        x_begin_array[i-3]=current_number;
        x_begin_shuzu[i-3]=current_number;
      }
    for (int j = 7; j < 10; j++)
      {
        in >> current_number;
        x_end.push_back(current_number);
        x_end_array[j-7]=current_number;
        x_end_shuzu[j-7]=current_number;
      }
    for (int k = 0; k < 3; k++)
      {
        for (int l = 0; l < 6100; l++)
        {
          in >> current_number;
          x_position[k].push_back(current_number);
        }
      }
    in.close();//关闭文件

  //运行SEDS模型
    int d=3;float T=1/256;
    MathLib::Vector x,xd,xT,xT_second,x_endGMM,x_go; //defining the required variables
    x.Resize(d); //d is the dimensionality of  your model
    xd.Resize(d);

  //定义目标位置xT	
    for (int i = 0; i < 3; i++)
      {
        current_number=x_begin_shuzu[i];
        xT[i]=current_number;
      }
    // //查看向量传递结果
    // cout << "The xT are: ";
    // for (int count = 0; count < 3; count++){
    //   cout << xT[count] << "; ";
    //   }

  //定义第二个目标位置xT_second	也就是运动最终停止位置
      xT_second[0]=0.421120645169;
      xT_second[1]=0.0205836859991;
      xT_second[2]=0.581226265672;      

    // //查看向量传递结果
    // cout << "The xT_second are: ";
    // for (int count = 0; count < 3; count++){
    //   cout << xT_second[count] << "; ";
    //   }

  //定义GMM结束位置x_endGMM	
    for (int i = 0; i < 3; i++)
      {
        current_number=x_end_shuzu[i];
        x_endGMM[i]=current_number;
      }
    // //查看向量传递结果
    // cout << "The x_endGMM are: ";
    // for (int count = 0; count < 3; count++){
    //   cout << x_endGMM[count] << "; ";
    //   }
    float xx1,xx2,xx3;
//ROS循环
while (ros::ok())
  {
  //定义当前位置x
  
    xx1=pose_go.pose.position.x;
    xx2=pose_go.pose.position.y;
    xx3=pose_go.pose.position.z;
    x[0]=xx1;
    x[1]=xx2;
    x[2]=xx3;
    // //测试用，直接输入是bag中的数据，实际中要输入机器人末端实时位置
    // x[0]=msg.pose.position.x;
    // x[1]=msg.pose.position.y;
    // x[2]=msg.pose.position.z;
    //查看向量传递结果
    cout << "The x are: ";
    for (int count = 0; count < 3; count++){
      cout << x[count] << "; ";
      }
    cout << "The xx1 are: ";
    cout << xx1 << "; ";

  //基于位置和速度判断seds是否执行结束
    if(pose_comd.pose.position.y-xT[1]<-0.001 && pose_comd.pose.position.z-xT[2]<-0.001)//差值是负数说明还在SEDS范围中
    {
        if((xd[1]+xd[2]+xd[3])/3>0.0003)//速度较大就说明SEDS没运行完
        {
        //运行模型回归
          x -= xT; //Transformation into the target frame of  reference
          mySEDS.doRegression(x,xd);  // Estimating xd at x
          xd.Print("xd = "); //Printing the value of  xd
        //给机器人速度与阻抗命令
          x_go[0]=x[0]+xd[0]*T;
          x_go[1]=x[1]+xd[1]*T;
          x_go[2]=x[2]+xd[2]*T;
        //传递给publisher
          pose_comd.pose.position.x=x_go[0];
          pose_comd.pose.position.y=x_go[1];
          pose_comd.pose.position.z=x_go[2];
          pose_comd.pose.orientation.x=msg.pose.orientation.x;
          pose_comd.pose.orientation.y=msg.pose.orientation.y;
          pose_comd.pose.orientation.z=msg.pose.orientation.z;
          pose_comd.pose.orientation.w=msg.pose.orientation.w;
        //发送位置与阻抗命令
          pose_command_pub.publish(pose_comd);
          kx=10;
          ky=10;
          kz=10;
          set_impedance(config,pt_mode);
          client.call(config);
          ROS_INFO("%f,%f,%f", kx,ky,kz);
          pose_go_pub.publish(pose_go);
          force_go_pub.publish(force_go);
        }
        else//速度小了说明接近目标了，就可以直接一个命令走到命令位置，这样差值讲接近零而不符合第一个if的判断从而进入GMM
        {
          pose_comd.pose.position.x=xT[0];
          pose_comd.pose.position.y=xT[1];
          pose_comd.pose.position.z=xT[2];
          pose_comd.pose.orientation.x=msg.pose.orientation.x;
          pose_comd.pose.orientation.y=msg.pose.orientation.y;
          pose_comd.pose.orientation.z=msg.pose.orientation.z;
          pose_comd.pose.orientation.w=msg.pose.orientation.w;
        //发送位置与阻抗命令
          pose_command_pub.publish(pose_comd);
          kx=10;
          ky=10;
          kz=10;
          set_impedance(config,pt_mode);
          client.call(config);
          ROS_INFO("%f,%f,%f", kx,ky,kz);
          pose_go_pub.publish(pose_go);
          force_go_pub.publish(force_go);
        }
    }
    else if(pose_comd.pose.position.y-x_endGMM[1]<-0.001 && pose_comd.pose.position.z-x_endGMM[2]<-0.001)//说明此时差值大于-0，001，以走完seds，判断如果没接近GMM终点
          {
            if((pose_comd.pose.position.y-x_endGMM[1])*(pose_comd.pose.position.y-x_endGMM[1])+(pose_comd.pose.position.z-x_endGMM[2])*(pose_comd.pose.position.z-x_endGMM[2])>0.001)//判断与终点的绝对位置
            {
              int i=0;
              pose_comd.pose.position.x=x_position[0][i];
              pose_comd.pose.position.y=x_position[1][i];
              pose_comd.pose.position.z=x_position[2][i];
              pose_comd.pose.orientation.x=msg.pose.orientation.x;
              pose_comd.pose.orientation.y=msg.pose.orientation.y;
              pose_comd.pose.orientation.z=msg.pose.orientation.z;
              pose_comd.pose.orientation.w=msg.pose.orientation.w;
              i=i+1;
            //发送位置与阻抗命令
              pose_command_pub.publish(pose_comd);
              kx=10;
              ky=10;
              kz=10;
              set_impedance(config,pt_mode);
              client.call(config);
              ROS_INFO("%f,%f,%f", kx,ky,kz);
              pose_go_pub.publish(pose_go);
              force_go_pub.publish(force_go);
            }
            else//如果接近终点了就直接走到终点
            {
              pose_comd.pose.position.x=x_endGMM[0];
              pose_comd.pose.position.y=x_endGMM[1];
              pose_comd.pose.position.z=x_endGMM[2];
              pose_comd.pose.orientation.x=msg.pose.orientation.x;
              pose_comd.pose.orientation.y=msg.pose.orientation.y;
              pose_comd.pose.orientation.z=msg.pose.orientation.z;
              pose_comd.pose.orientation.w=msg.pose.orientation.w;
            //发送位置与阻抗命令
              pose_command_pub.publish(pose_comd);
              kx=30;
              kx=30;
              kx=30;
              set_impedance(config,pt_mode);
              client.call(config);
              ROS_INFO("%f,%f,%f", kx,ky,kz);
              pose_go_pub.publish(pose_go);
              force_go_pub.publish(force_go);
            }
          }
        else //走到了GMM终点，于是走第二个seds
        {
        //运行模型回归
          x -= xT_second; //Transformation into the target frame of  reference
          mySEDS_second.doRegression(x,xd);  // Estimating xd at x
          xd.Print("xd = "); //Printing the value of  xd
        //给机器人速度与阻抗命令
          x_go[0]=x[0]+xd[0]*T;
          x_go[1]=x[1]+xd[1]*T;
          x_go[2]=x[2]+xd[2]*T;
        //传递给publisher
          pose_comd.pose.position.x=x_go[0];
          pose_comd.pose.position.y=x_go[1];
          pose_comd.pose.position.z=x_go[2];
          pose_comd.pose.orientation.x=msg.pose.orientation.x;
          pose_comd.pose.orientation.y=msg.pose.orientation.y;
          pose_comd.pose.orientation.z=msg.pose.orientation.z;
          pose_comd.pose.orientation.w=msg.pose.orientation.w;
        //发送位置与阻抗命令
          pose_command_pub.publish(pose_comd);
          kx=10;
          ky=10;
          kz=10;
          set_impedance(config,pt_mode);
          client.call(config);
          ROS_INFO("%f,%f,%f", kx,ky,kz);
          pose_go_pub.publish(pose_go);
          force_go_pub.publish(force_go);
        }
   
    ros::spinOnce();

    loop_rate.sleep();
   
    }
  return 0;
}

void set_impedance(iiwa_msgs::ConfigureSmartServo& config1,int& pt)
{
  if(pt==1){
  config1.request.mode.mode = iiwa_msgs::SmartServoMode::CARTESIAN_IMPEDANCE;
  config1.request.mode.cartesian_stiffness.stiffness.x = 200; 
  config1.request.mode.cartesian_stiffness.stiffness.y = 200; 
  config1.request.mode.cartesian_stiffness.stiffness.z = 200; 
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
  kx= config1.request.mode.cartesian_stiffness.stiffness.x;
  ky= config1.request.mode.cartesian_stiffness.stiffness.y;
  kz= config1.request.mode.cartesian_stiffness.stiffness.z;
  ROS_INFO("%f,%f,%f", kx,ky,kz);
   }
  else if(pt==2)
{
  config1.request.mode.mode = iiwa_msgs::SmartServoMode::CARTESIAN_IMPEDANCE;
  config1.request.mode.cartesian_stiffness.stiffness.x = 200; 
  config1.request.mode.cartesian_stiffness.stiffness.y = 200; 
  config1.request.mode.cartesian_stiffness.stiffness.z = 500; 
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
  kx= config1.request.mode.cartesian_stiffness.stiffness.x;
  ky= config1.request.mode.cartesian_stiffness.stiffness.y;
  kz= config1.request.mode.cartesian_stiffness.stiffness.z;
  ROS_INFO("%f,%f,%f", kx,ky,kz);
}
  else if(pt==3&&kz>0)
{
  config1.request.mode.mode = iiwa_msgs::SmartServoMode::CARTESIAN_IMPEDANCE;
  config1.request.mode.cartesian_stiffness.stiffness.x = kx; 
  config1.request.mode.cartesian_stiffness.stiffness.y = ky; 
  config1.request.mode.cartesian_stiffness.stiffness.z = kz; 
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
}
}
/*
  position: 
    x: 0.421120645169
    y: 0.0205836859991
    z: 0.581226265672
  orientation: 
    x: -0.353685581886
    y: 0.935302317142
    z: -0.00418645251225
    w: 0.00992877288998*/
