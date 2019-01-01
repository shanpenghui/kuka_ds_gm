#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include "iiwaRos.h"
#include "geometry_msgs/PoseStamped.h"
geometry_msgs::PoseStamped start_pose,end_pose,now_pose;
void set_pose(geometry_msgs::PoseStamped&,double,double,double);
void now_pose_callback(const geometry_msgs::PoseStamped& sub_msg0);
int goal_flg=0;
double mx=1,my=1,mz=1,nx=1,ny=1,nz=1;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pt_recircle");
  ros::NodeHandle n;
  set_pose(start_pose,0.421,0.02,0.7);
  set_pose(end_pose,0.42,0.02,0.7);
  ros::Publisher pose_pub = n.advertise<geometry_msgs::PoseStamped>("/iiwa/command/CartesianPose", 1000);
  ros::Subscriber pose_sub = n.subscribe("/iiwa/state/CartesianPose", 1000, now_pose_callback);
  ros::Rate loop_rate(100);
  while (ros::ok()) {
  
    if(goal_flg==0)
    pose_pub.publish(start_pose);
    else
   pose_pub.publish(end_pose); 
   
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
void set_pose(geometry_msgs::PoseStamped& msg,double X,double Y,double Z)
{
    msg.pose.position.x=X;
    msg.pose.position.y=Y;
    msg.pose.position.z=Z;

    msg.pose.orientation.x=-0.353685581886;
    msg.pose.orientation.y=0.935302317142;
    msg.pose.orientation.z=-0.00418645251225;
    msg.pose.orientation.w=0.0099287728899;
    

   ROS_INFO("command_pose:%f,%f,%f",msg.pose.position.x,msg.pose.position.y,msg.pose.position.z);

}
void now_pose_callback(const geometry_msgs::PoseStamped& sub_msg0)
{
  now_pose.pose.position.x=sub_msg0.pose.position.x;
  now_pose.pose.position.y=sub_msg0.pose.position.y;
  now_pose.pose.position.z=sub_msg0.pose.position.z;
  
    if(goal_flg==0){
    mx=start_pose.pose.position.x-now_pose.pose.position.x;
    my=start_pose.pose.position.y-now_pose.pose.position.y;
    mz=start_pose.pose.position.z-now_pose.pose.position.z;
   if(mx<0)mx=-mx;
   if(my<0)my=-my;
   if(mz<0)mz=-mz;
   if(mx<0.0005&&my<0.0005&&mz<0.0005)goal_flg=1;
   else goal_flg=0;
   }
    if(goal_flg==1){
    nx=end_pose.pose.position.x-now_pose.pose.position.x;
    ny=end_pose.pose.position.y-now_pose.pose.position.y;
    nz=end_pose.pose.position.z-now_pose.pose.position.z;
     if(nx<0)nx=-nx;
     if(ny<0)ny=-ny;
     if(nz<0)nz=-nz;
      if(nx<0.0005&&ny<0.0005&&nz<0.0005)goal_flg=1;
       else goal_flg=1;
    }
  ROS_INFO("now_pose:%d,%f,%f,%f,%f,%f,%f,%f,%f",goal_flg,mz, sub_msg0.pose.position.x,sub_msg0.pose.position.y,sub_msg0.pose.position.z,sub_msg0.pose.orientation.x,sub_msg0.pose.orientation.y,sub_msg0.pose.orientation.z,sub_msg0.pose.orientation.w);

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
