#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <iostream>
#include <vector>
#include "iiwaRos.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"
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
int SEDSstage=0;
int pt=1,flg=1,pt_flg=1,call_ready=1;//为阶段变化阻抗设定的值
geometry_msgs::PoseStamped msg;
geometry_msgs::PoseStamped pose_comd;
// geometry_msgs::PoseStamped vol_comd;
geometry_msgs::Pose stiff_comd;
geometry_msgs::PoseStamped pose_go;
geometry_msgs::WrenchStamped force_go;
geometry_msgs::PoseStamped pose_go_commend;
std_msgs::Float64 pt_wrench_x;
std_msgs::Float64 pt_wrench_y;
std_msgs::Float64 pt_wrench_z;
void set_impedance(iiwa_msgs::ConfigureSmartServo& config1,int& pt);
// void pose_chatterCallback(const geometry_msgs::PoseStamped& msg1)
// {
//   msg.pose.position.x=msg1.pose.position.x;
//   msg.pose.position.y=msg1.pose.position.y;
//   msg.pose.position.z=msg1.pose.position.z;
//   msg.pose.orientation.x=msg1.pose.orientation.x;
//   msg.pose.orientation.y=msg1.pose.orientation.y;
//   msg.pose.orientation.z=msg1.pose.orientation.z;
//   msg.pose.orientation.w=msg1.pose.orientation.w;
// }
void pose_iiwa_Callback(const geometry_msgs::PoseStamped& msg1)
{
  pose_go.pose.position.x=msg1.pose.position.x;
  pose_go.pose.position.y=msg1.pose.position.y;
  pose_go.pose.position.z=msg1.pose.position.z;
  // pose_go.pose.orientation.x =msg1.pose.orientation.x;
  // pose_go.pose.orientation.y =msg1.pose.orientation.y;
  // pose_go.pose.orientation.z =msg1.pose.orientation.z;
  // pose_go.pose.orientation.w =msg1.pose.orientation.w;
  pose_go.pose.orientation.x=2.24082595912e-05;
  pose_go.pose.orientation.y=0.999999463558;
  pose_go.pose.orientation.z=1.84833353326e-05;
  pose_go.pose.orientation.w=-0.00104649211296;
  // ROS_INFO("%f", msg1.pose.position.x);

    if (SEDSstage==2)
    {
      pt=0;
    }
    else
    {
      pt=1;
    }
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
  ros::init(argc, argv, "wr_SEDS_GMM");
  ros::NodeHandle n;

//接受机器人状态
  // ros::Subscriber pose_sub = n.subscribe("/pose", 100, pose_chatterCallback);//要加一个运行包，不断发送姿态命令，这样就不用自己编姿态了
  ros::Subscriber pose_iiwa_sub = n.subscribe("/iiwa/state/CartesianPose", 1000, pose_iiwa_Callback);
  ros::Subscriber wrench_iiwa_sub = n.subscribe("/iiwa/state/CartesianWrench", 1000, wrench_iiwa_callback);

//发布控制模式
  ros::ServiceClient client = n.serviceClient<iiwa_msgs::ConfigureSmartServo>("/iiwa/configuration/configureSmartServo");

//发布控制命令
  ros::Publisher pose_command_pub = n.advertise<geometry_msgs::PoseStamped>("/iiwa/command/CartesianPose", 1000);
  // ros::Publisher vol_command_pub = n.advertise<geometry_msgs::PoseStamped>("/iiwa/command/CartesianVelocity", 1000);
  ros::Publisher stiff_command_pub = n.advertise<geometry_msgs::Pose>("CartesianStiffness", 1000);

// 发布绘图话题
  ros::Publisher pose_go_pub = n.advertise<geometry_msgs::PoseStamped>("pose_go", 1000);        //绘制当前实际位置
  ros::Publisher force_go_pub = n.advertise<geometry_msgs::WrenchStamped>("force_go", 1000);
  // ros::Publisher pose_go_commend_pub = n.advertise<geometry_msgs::PoseStamped>("pose_go_commend", 1000);   //绘制当前命令值

  iiwa_msgs::ConfigureSmartServo config;
  ros::Rate loop_rate(1000);
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
    std::vector<float> x_begin;
    std::vector<float> x_end;
    std::array<float,3> x_begin_array;
    std::array<float,3> x_end_array;
    float x_begin_shuzu[3];
    float x_end_shuzu[3];
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

  //设定SEDS模型运算时的变量
    int d=3;float T=1.0/15.0;float T1=1.0/1000.0;float T2=1.0/5.0;float T3=1.0/2.0;float T4=1.0/1.0; int GMMtime=0; float xd_mean1=3.0;
    MathLib::Vector x,x_xiangdui,xd,xT,xT_second,x_endGMM,x_go,x_position_before,xd1,xd2,xd3,xd4,xd5; //defining the required variables
    x.Resize(d); //d is the dimensionality of  your model
    xd.Resize(d);xd1.Resize(d);xd2.Resize(d);xd3.Resize(d);xd4.Resize(d);xd5.Resize(d);x_xiangdui.Resize(d);
    xT.Resize(d);
    xT_second.Resize(d);
    x_endGMM.Resize(d);
    x_go.Resize(d);
    x_position_before.Resize(d);

  //定义目标位置xT
    for (int i = 0; i < 3; i++)
      {
        current_number=x_begin_shuzu[i];
        xT[i]=current_number;
      }
      x_position_before=xT;
      // xT[0]=0.611509;
      // xT[1]=0.020930;
      // xT[2]=0.223562;
    //查看向量传递结果
    // cout << "The xT are: ";    //传递给publisher
    // for (int count = 0; count < 3; count++){
    //   cout << xT[count] << "; ";
    //   }
    //xT.Print("xT = ");

  //定义第二个目标位置xT_second	也就是运动最终停止位置
      xT_second[0]=0.421120645169;
      xT_second[1]=0.0205836859991;
      xT_second[2]=0.581226265672;

    //查看向量传递结果
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
    //查看向量传递结果
    // cout << "The x_endGMM are: ";
    // for (int count = 0; count < 3; count++){
    //   cout << x_endGMM[count] << "; ";
    //   }
    float xx1,xx2,xx3;


//ROS循环
while (ros::ok())
  {

  //首先定义阻抗的分阶段变化判断语句
  set_impedance(config,pt);
   if(call_ready==1)
    if (client.call(config)) {
        if(!config.response.success)
            ROS_ERROR_STREAM("Config failed, Java error: " << config.response.error);
        else {
            if(pt==0&&flg==0){
            ROS_INFO_STREAM("High impedance successfully set.");flg=1;call_ready=0;}
            if(pt==1&&flg==1){
            ROS_INFO_STREAM("Low impedance successfully set.");flg=0;call_ready=0;}
          }
      }
    else {
        ROS_ERROR_STREAM("Config failed - service could not be called - QUITTING NOW !");
      }




  //定义当前位置x
    pose_go_pub.publish(pose_go);
    force_go_pub.publish(force_go);

    xx1=pose_go.pose.position.x;
    xx2=pose_go.pose.position.y;
    xx3=pose_go.pose.position.z;
    x[0]=xx1;
    x[1]=xx2;
    x[2]=xx3;
    // x[0]=0.57848316431;
    // x[1]=0.000486766191898;
    // x[2]=0.471556246281;
    // //测试用，直接输入是bag中的数据，实际中要输入机器人末端实时位置
    // x[0]=msg.pose.position.x;
    // x[1]=msg.pose.position.y;
    // x[2]=msg.pose.position.z;
    //查看向量传递结果

   // x.Print("xold = ");

    // cout << "The x are: ";
    // for (int count = 0; count < 3; count++){
    //   cout << x[count] << "; ";
    //   }

    //       cout << "The xT are: ";
    // for (int count = 0; count < 3; count++){
    //   cout << xT[count] << "; ";
    //   }

    // cout << "The xx1 are: ";
    // cout << xx1 << "; ";

  //基于位置和速度判断seds是否执行结束

    if (SEDSstage==0)
    {
      //0步设置是因为计算初始有很大的速度，机械臂会自动停止，所以先让程序计算一段时间速度，之后速度小了就运行1
        cout << "  zero step; \n ";
      //运行模型回归
        x_xiangdui=x - xT; //Transformation into the target frame of  reference   x -= xT;不成功，原因是后面x_go 的计算用到了x;
        //x.Print("x = ");
        mySEDS.doRegression(x_xiangdui,xd);  // Estimating xd at x
        xd.Print("xd = "); //Printing the value of  xd
      //给机器人速度与阻抗命令
        x_go[0]=x[0]+xd[0]*T;
        x_go[1]=x[1]+xd[1]*T;
        x_go[2]=x[2]+xd[2]*T;

      //对SEDS进行的状态进行判断，
      //如果速度进入正常范围，则开始运动
        float xd_mean0=sqrt(xd[0]*xd[0])+sqrt(xd[1]*xd[1])+sqrt(xd[2]*xd[2]);
        cout << " the mean of vol: ";
        cout << xd_mean0 << "; \n";
        if(xd_mean0<6)
        {
          SEDSstage=1;
          cout << "  change to first step; \n ";
        }
    }


  if (SEDSstage==1)
  {
  //如果seds运行未完成，则执行seds

      cout << "  first step; \n ";
    //运行模型回归
      x_xiangdui=x - xT; //Transformation into the target frame of  reference   x -= xT;不成功，原因是后面x_go 的计算用到了x;
      //x.Print("x = ");
      mySEDS.doRegression(x_xiangdui,xd);  // Estimating xd at x
      xd.Print("xd = "); //Printing the value of  xd
    //给机器人速度与阻抗命令
      x_go[0]=x[0]+xd[0]*T;
      x_go[1]=x[1]+xd[1]*T;
      x_go[2]=x[2]+xd[2]*T;

          //   //查看当前位置
          // cout << "x are: ";
          // for (int count = 0; count < 3; count++){
          // cout << x[count] << "; ";
          //     }
          //   //查看位置命令
          // cout << "x_go are: ";
          // for (int count = 0; count < 3; count++){
          // cout << x_go[count] << "; ";
          //     }


    //传递给publisher
      pose_comd.pose.position.x=x_go[0];
      pose_comd.pose.position.y=x_go[1];
      pose_comd.pose.position.z=x_go[2];
      // pose_comd.pose.orientation.x=msg.pose.orientation.x;
      // pose_comd.pose.orientation.y=msg.pose.orientation.y;
      // pose_comd.pose.orientation.z=msg.pose.orientation.z;
      // pose_comd.pose.orientation.w=msg.pose.orientation.w;
      pose_comd.pose.orientation.x=pose_go.pose.orientation.x;
      pose_comd.pose.orientation.y=pose_go.pose.orientation.y;
      pose_comd.pose.orientation.z=pose_go.pose.orientation.z;
      pose_comd.pose.orientation.w=pose_go.pose.orientation.w;
    //为了仿真设置的速度
    //   vol_comd.pose.position.x=xd[0];
    //   vol_comd.pose.position.y=xd[1];
    //   vol_comd.pose.position.z=xd[2];
    //   vol_command_pub.publish(vol_comd);


//     //发送位置与阻抗命令
      pose_command_pub.publish(pose_comd);
// //阻抗的设定
//       kx=10;
//       ky=10;
//       kz=10;
//       set_impedance(config,pt_mode);
//       client.call(config);
      stiff_comd.position.x=300;
      stiff_comd.position.y=300;
      stiff_comd.position.z=300;
      stiff_command_pub.publish(stiff_comd);
//       // ROS_INFO("%f,%f,%f", kx,ky,kz);
      // pose_go_pub.publish(pose_go);
      // force_go_pub.publish(force_go);



    //对SEDS进行的状态进行判断，
    //如果距离小于某个值之后，就可以执行位置命令并给SEDSstage加一
      float xd_mean1=sqrt(xd[0]*xd[0])+sqrt(xd[1]*xd[1])+sqrt(xd[2]*xd[2]);
      cout << " the mean of vol: ";
      cout << xd_mean1 << "; \n";
      if(xd_mean1<0.33)//距离远说明还在SEDS范围中   //sqrt(pose_comd.pose.position.x-xT[0])+sqrt(pose_comd.pose.position.z-xT[2])<-0.0000000001
      {
        T=T2;
      }
      if(xd_mean1<0.23)//距离远说明还在SEDS范围中   //sqrt(pose_comd.pose.position.x-xT[0])+sqrt(pose_comd.pose.position.z-xT[2])<-0.0000000001
      {
        T=T3;
      }
      // if(xd_mean1<0.13)//距离远说明还在SEDS范围中   //sqrt(pose_comd.pose.position.x-xT[0])+sqrt(pose_comd.pose.position.z-xT[2])<-0.0000000001
      // {
      //   T=T4;
      // }
      if(xd_mean1<0.018)//距离远说明还在SEDS范围中   //sqrt(pose_comd.pose.position.x-xT[0])+sqrt(pose_comd.pose.position.z-xT[2])<-0.0000000001
      {
        SEDSstage=2;
        cout << "  change to second step; \n ";
      }
  }

//   if (SEDSstage==9)
//   {
//   //如果seds运行未完成，则执行seds

//       cout << "  9 step; \n ";
//     //运行模型回归
//       x_xiangdui=x - xT; //Transformation into the target frame of  reference   x -= xT;不成功，原因是后面x_go 的计算用到了x;
//       //x.Print("x = ");
//       mySEDS.doRegression(x_xiangdui,xd);  // Estimating xd at x
//       xd.Print("xd = "); //Printing the value of  xd
//     //给机器人速度与阻抗命令
//       x_go[0]=x[0]+xd[0]*T2;
//       x_go[1]=x[1]+xd[1]*T2;
//       x_go[2]=x[2]+xd[2]*T2;

//           //   //查看当前位置
//           // cout << "x are: ";
//           // for (int count = 0; count < 3; count++){
//           // cout << x[count] << "; ";
//           //     }
//           //   //查看位置命令
//           // cout << "x_go are: ";
//           // for (int count = 0; count < 3; count++){
//           // cout << x_go[count] << "; ";
//           //     }


//     //传递给publisher
//       pose_comd.pose.position.x=x_go[0];
//       pose_comd.pose.position.y=x_go[1];
//       pose_comd.pose.position.z=x_go[2];
//       // pose_comd.pose.orientation.x=msg.pose.orientation.x;
//       // pose_comd.pose.orientation.y=msg.pose.orientation.y;
//       // pose_comd.pose.orientation.z=msg.pose.orientation.z;
//       // pose_comd.pose.orientation.w=msg.pose.orientation.w;
//       pose_comd.pose.orientation.x=pose_go.pose.orientation.x;
//       pose_comd.pose.orientation.y=pose_go.pose.orientation.y;
//       pose_comd.pose.orientation.z=pose_go.pose.orientation.z;
//       pose_comd.pose.orientation.w=pose_go.pose.orientation.w;
//     //为了仿真设置的速度
//     //   vol_comd.pose.position.x=xd[0];
//     //   vol_comd.pose.position.y=xd[1];
//     //   vol_comd.pose.position.z=xd[2];
//     //   vol_command_pub.publish(vol_comd);


// //     //发送位置与阻抗命令
//       pose_command_pub.publish(pose_comd);
// // //阻抗的设定
// //       kx=10;
// //       ky=10;
// //       kz=10;
// //       set_impedance(config,pt_mode);
// //       client.call(config);
//       stiff_comd.position.x=300;
//       stiff_comd.position.y=300;
//       stiff_comd.position.z=300;
//       stiff_command_pub.publish(stiff_comd);
// //       // ROS_INFO("%f,%f,%f", kx,ky,kz);
//       // pose_go_pub.publish(pose_go);
//       // force_go_pub.publish(force_go);



//     //对SEDS进行的状态进行判断，
//     //如果距离小于某个值之后，就可以执行位置命令并给SEDSstage加一
//       float xd_mean1=sqrt(xd[0]*xd[0])+sqrt(xd[1]*xd[1])+sqrt(xd[2]*xd[2]);
//       cout << " the mean of vol: ";
//       cout << xd_mean1 << "; \n";
//       if(xd_mean1<0.09)//距离远说明还在SEDS范围中   //sqrt(pose_comd.pose.position.x-xT[0])+sqrt(pose_comd.pose.position.z-xT[2])<-0.0000000001
//       {
//         SEDSstage=2;
//         cout << "  change to second step; \n ";
//       }
//   }


//若走到GMM起始点，就开始GMM
  if (SEDSstage==2)
  {
    cout << "  second step; \n ";
      if (GMMtime<6099)
      {
        GMMtime +=1;
      }
      else
      {
        GMMtime==6099;
      }

      pose_comd.pose.position.x=x_position[0][GMMtime];
      pose_comd.pose.position.y=x_position[1][GMMtime];
      pose_comd.pose.position.z=x_position[2][GMMtime];
      // pose_comd.pose.orientation.x=msg.pose.orientation.x;
      // pose_comd.pose.orientation.y=msg.pose.orientation.y;
      // pose_comd.pose.orientation.z=msg.pose.orientation.z;
      // pose_comd.pose.orientation.w=msg.pose.orientation.w;
      pose_comd.pose.orientation.x=pose_go.pose.orientation.x;
      pose_comd.pose.orientation.y=pose_go.pose.orientation.y;
      pose_comd.pose.orientation.z=pose_go.pose.orientation.z;
      pose_comd.pose.orientation.w=pose_go.pose.orientation.w;

      xd2[0]=(x_position[0][GMMtime]-x_position_before[0])/T1;
      xd2[1]=(x_position[1][GMMtime]-x_position_before[1])/T1;
      xd2[2]=(x_position[2][GMMtime]-x_position_before[2])/T1;
      x_position_before[0]=x_position[0][GMMtime];
      x_position_before[1]=x_position[1][GMMtime];
      x_position_before[2]=x_position[2][GMMtime];
      xd2.Print("xd2 = ");

        //为了仿真设置的速度

    //   vol_comd.pose.position.x=xd2[0];
    //   vol_comd.pose.position.y=xd2[1];
    //   vol_comd.pose.position.z=xd2[2];
    //   vol_command_pub.publish(vol_comd);


//     //发送位置与阻抗命令
      pose_command_pub.publish(pose_comd);
//  //阻抗参数
//       kx=300;
//       ky=300;
//       kz=300;
//       set_impedance(config,pt_mode);
//       client.call(config);
      stiff_comd.position.x=1000;
      stiff_comd.position.y=1000;
      stiff_comd.position.z=1000;
      stiff_command_pub.publish(stiff_comd);
//       //ROS_INFO("%f,%f,%f", kx,ky,kz);
      // pose_go_pub.publish(pose_go);
      // force_go_pub.publish(force_go);


      //判断到没到这个点，到了就改状态
      float xd_mean2=sqrt(xd2[0]*xd2[0])+sqrt(xd2[1]*xd2[1])+sqrt(xd2[2]*xd2[2]);
      cout << " the mean of vol2: ";
      cout << xd_mean2 << "; \n";
      if(xd_mean2<0.000682)//距离远说明还在SEDS范围中   //sqrt(pose_comd.pose.position.x-xT[0])+sqrt(pose_comd.pose.position.z-xT[2])<-0.0000000001
      {
        SEDSstage=3;
        cout << "  change to thrid step;  \n";
      }
  }


  if (SEDSstage==3)
  {
          cout << "  third step;  \n";
        //运行模型回归
          x_xiangdui=x - xT_second; //Transformation into the target frame of  reference
          mySEDS_second.doRegression(x_xiangdui,xd3);  // Estimating xd at x
          xd3.Print("xd3 = "); //Printing the value of  xd
        //给机器人速度与阻抗命令
          x_go[0]=x[0]+xd3[0]*T;
          x_go[1]=x[1]+xd3[1]*T;
          x_go[2]=x[2]+xd3[2]*T;
        //传递给publisher
          pose_comd.pose.position.x=x_go[0];
          pose_comd.pose.position.y=x_go[1];
          pose_comd.pose.position.z=x_go[2];
          // pose_comd.pose.orientation.x=msg.pose.orientation.x;
          // pose_comd.pose.orientation.y=msg.pose.orientation.y;
          // pose_comd.pose.orientation.z=msg.pose.orientation.z;
          // pose_comd.pose.orientation.w=msg.pose.orientation.w;
          pose_comd.pose.orientation.x=pose_go.pose.orientation.x;
          pose_comd.pose.orientation.y=pose_go.pose.orientation.y;
          pose_comd.pose.orientation.z=pose_go.pose.orientation.z;
          pose_comd.pose.orientation.w=pose_go.pose.orientation.w;

       //为了仿真设置的速度
        //   vol_comd.pose.position.x=xd3[0];
        //   vol_comd.pose.position.y=xd3[1];
        //   vol_comd.pose.position.z=xd3[2];
        //   vol_command_pub.publish(vol_comd);


      //   //发送位置与阻抗命令
          pose_command_pub.publish(pose_comd);
      //  //阻抗参数
      //     kx=10;
      //     ky=10;
      //     kz=10;
      //     set_impedance(config,pt_mode);
      //     client.call(config);
          stiff_comd.position.x=300;
          stiff_comd.position.y=300;
          stiff_comd.position.z=300;
          stiff_command_pub.publish(stiff_comd);
      //     //ROS_INFO("%f,%f,%f", kx,ky,kz);
          // pose_go_pub.publish(pose_go);
          // force_go_pub.publish(force_go);




                    //判断到没到这个点，到了就改状态
      float xd_mean3=sqrt(xd3[0]*xd3[0])+sqrt(xd3[1]*xd3[1])+sqrt(xd3[2]*xd3[2]);
      cout << " the mean of vol3: ";
      cout << xd_mean3 << "; \n";
      if(xd_mean3<0.142)//距离远说明还在SEDS范围中   //sqrt(pose_comd.pose.position.x-xT[0])+sqrt(pose_comd.pose.position.z-xT[2])<-0.0000000001
      {
        SEDSstage=4;
      }
  }

  if (SEDSstage==4)
  {
    cout << "  finish the task!  \n";
  }

    ros::spinOnce();

    loop_rate.sleep();

    }
  return 0;
 }

void set_impedance(iiwa_msgs::ConfigureSmartServo& config1,int& pt1)
{
 if(pt1==0&&pt_flg==0){
  config1.request.mode.mode = iiwa_msgs::SmartServoMode::CARTESIAN_IMPEDANCE;

  config1.request.mode.cartesian_stiffness.stiffness.x = 1000;
  config1.request.mode.cartesian_stiffness.stiffness.y = 1000;
  config1.request.mode.cartesian_stiffness.stiffness.z = 1000;
  config1.request.mode.cartesian_stiffness.stiffness.a =1000;
  config1.request.mode.cartesian_stiffness.stiffness.b =1000;
  config1.request.mode.cartesian_stiffness.stiffness.c =1000;

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
  config1.request.mode.cartesian_stiffness.stiffness.x = 300;
  config1.request.mode.cartesian_stiffness.stiffness.y = 300;
  config1.request.mode.cartesian_stiffness.stiffness.z = 300;
  config1.request.mode.cartesian_stiffness.stiffness.a = 300;
  config1.request.mode.cartesian_stiffness.stiffness.b = 300;
  config1.request.mode.cartesian_stiffness.stiffness.c = 300;

  config1.request.mode.cartesian_damping.damping.x = 0.1;
  config1.request.mode.cartesian_damping.damping.y = 0.1;
  config1.request.mode.cartesian_damping.damping.z = 0.1;
  config1.request.mode.cartesian_damping.damping.a = 0.1;
  config1.request.mode.cartesian_damping.damping.b = 0.1;
  config1.request.mode.cartesian_damping.damping.c = 0.1;

  config1.request.mode.nullspace_stiffness =10;
  config1.request.mode.nullspace_damping = 1;
  pt_flg=0,call_ready=1;
  }

else{;}

}
