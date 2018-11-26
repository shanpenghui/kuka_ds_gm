#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/WrenchStamped.h"
geometry_msgs::PoseStamped pub_msg,pub_start_pose;
geometry_msgs::PoseStamped sub_pose_msg;
geometry_msgs::WrenchStamped sub_wrench_msg;
int count=0,start=0,ready_move=0;
double force_X[5]={};
double force_Y[5]={};
double force_Z[5]={};
double torque_X[5]={};
double torque_Y[5]={};
double torque_Z[5]={};
double torque_W[5]={};
const double minForce_limit=0.5;
const double minForce[6]={0.593066640637,1.43335560276,2.61525075229,0.237606608173,-0.460006794403,0.134757960982};
void sub_pose_callback(const geometry_msgs::PoseStamped& sub_msg0)
{
  sub_pose_msg.pose.position.x=sub_msg0.pose.position.x;
  sub_pose_msg.pose.position.y=sub_msg0.pose.position.y;
  sub_pose_msg.pose.position.z=sub_msg0.pose.position.z;
 // ROS_INFO("listener_pose:%f,%f,%f,%f,%f,%f,%f", sub_msg0.pose.position.x,sub_msg0.pose.position.y,sub_msg0.pose.position.z,sub_msg0.pose.orientation.x,sub_msg0.pose.orientation.y,sub_msg0.pose.orientation.z,sub_msg0.pose.orientation.w);

}
void sub_wrench_callback(const geometry_msgs::WrenchStamped& sub_msg1)
{
    if(count<5){
    force_X[count]=sub_msg1.wrench.force.x;
    force_Y[count]=sub_msg1.wrench.force.y;
    force_Z[count]=sub_msg1.wrench.force.z;
    torque_X[count]=sub_msg1.wrench.torque.x;
    torque_Y[count]=sub_msg1.wrench.torque.y;
    torque_Z[count]=sub_msg1.wrench.torque.z;
    }
    count++;
    if(count==5){
    count=0; 
       if((force_X[4]-minForce[0])<minForce_limit)
       sub_wrench_msg.wrench.force.x=0;
       else
        {sub_wrench_msg.wrench.force.x=(force_X[4]-minForce[0]);ready_move=1;}
       if((force_Y[4]- minForce[1])<minForce_limit)
       sub_wrench_msg.wrench.force.y=0;
       else
        {sub_wrench_msg.wrench.force.y=(force_Y[4]-minForce[1]);ready_move=1;}
       if((force_Z[4]-minForce[2])<minForce_limit)
       sub_wrench_msg.wrench.force.z=0;
       else
        {sub_wrench_msg.wrench.force.z=(force_Z[4]-minForce[2]);ready_move=1;}
       /*if((torque_X[4]-minForce[3])<minForce_limit)
       sub_wrench_msg.wrench.torque.x=0;
       else
        {sub_wrench_msg.wrench.torque.x=torque_X[4]-minForce[3];ready_move=1;}
       if((torque_Y[4]- minForce[4])<minForce_limit)
       sub_wrench_msg.wrench.torque.y=0;
       else
        {sub_wrench_msg.wrench.force.z=torque_Y[4]-minForce[4];ready_move=1;}
       if((torque_Z[4]-minForce[5])<minForce_limit)
       sub_wrench_msg.wrench.torque.z=0;
       else
        {sub_wrench_msg.wrench.torque.z=torque_Z[4]-minForce[5];ready_move=1; }*/
        
    }
  //ROS_INFO("listener_wrench:%f,%f,%f,%f,%f,%f", sub_msg1.wrench.force.x,sub_msg1.wrench.force.y,sub_msg1.wrench.force.z,sub_msg1.wrench.torque.x,sub_msg1.wrench.torque.y,sub_msg1.wrench.torque.z);
}
void set_pose(geometry_msgs::PoseStamped& pub_msg0,int K)
{
     
     pub_msg0.pose.position.x=sub_wrench_msg.wrench.force.x/K+sub_pose_msg.pose.position.x;
     pub_msg0.pose.position.y=sub_wrench_msg.wrench.force.y/K+sub_pose_msg.pose.position.y;
     pub_msg0.pose.position.z=sub_wrench_msg.wrench.force.z/K+sub_pose_msg.pose.position.z;
   //  pub_msg0.pose.orientation.x=sub_wrench_msg.wrench.torque.x/K+sub_pose_msg.pose.position.z;
   // pub_msg0.pose.position.y=0.02;
    //pub_msg0.pose.position.z=0.8;

    pub_msg0.pose.orientation.x=-0.353685581886;
    pub_msg0.pose.orientation.y=0.935302317142;
    pub_msg0.pose.orientation.z=-0.00418645251225;
    pub_msg0.pose.orientation.w=0.0099287728899;
    ROS_INFO("command_pose:%f,%f,%f,%f,%f,%f,%f", pub_msg0.pose.position.x,pub_msg0.pose.position.y,pub_msg0.pose.position.z,pub_msg0.pose.orientation.x,pub_msg0.pose.orientation.y,pub_msg0.pose.orientation.z,pub_msg0.pose.orientation.w);

}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "pt_impedance");

  
  ros::NodeHandle n;

  ros::Subscriber sub_pose = n.subscribe("/iiwa/state/CartesianPose", 1000, sub_pose_callback);
  ros::Subscriber sub_wrench = n.subscribe("/iiwa/state/CartesianWrench", 1000, sub_wrench_callback);
  ros::Publisher  pub_pose = n.advertise<geometry_msgs::PoseStamped>("/iiwa/command/CartesianPose", 1000);
  ros::Rate loop_rate(25);

 while (ros::ok())
  {
     if(ready_move==1){
    set_pose(pub_msg,1000);
   // pub_pose.publish(pub_msg);
     }
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
