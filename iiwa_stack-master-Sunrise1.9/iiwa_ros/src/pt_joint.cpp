#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include "iiwaRos.h"
#include "iiwa_msgs/JointPositionVelocity.h"

int main(int argc, char **argv)
{

 
  ros::init(argc, argv, "pt_joint");
  ros::NodeHandle n;

  ros::Publisher chatter_pub = n.advertise<iiwa_msgs::JointPositionVelocity>("/iiwa/command/JointPositionVelocity", 1000);

  ros::Rate loop_rate(10);

  int count = 0;
  while (ros::ok())
  {
    iiwa_msgs::JointPositionVelocity msg;

    /*msg.position.a1=0;
    msg.position.a2=0;
    msg.position.a3=0;
    msg.position.a4=0;
    msg.position.a5=0;
    msg.position.a6=0;
    msg.position.a7=0;*/

   msg.position.a1=0.151698589325;
    msg.position.a2=0.0610287673771;
    msg.position.a3=0.00989123992622;
    msg.position.a4=-1.83345043659;
    msg.position.a5=0.000470165163279;
    msg.position.a6=0.24445450306;
    msg.position.a7=0.160839855671;

    msg.velocity.a1=0.01;
    msg.velocity.a2=0.01;
    msg.velocity.a3=0.01;
    msg.velocity.a4=0.01;
    msg.velocity.a5=0.01;
    msg.velocity.a6= 0.1;
    msg.velocity.a7=0.01;
   ROS_INFO("%f,%f,%f,%f,%f,%f",msg.position.a1,msg.position.a2,msg.position.a3,msg.position.a4,msg.position.a5,msg.position.a6);

    chatter_pub.publish(msg);
   //my_iiwa_ros_object.setJointPosition(my_joint_position); 
    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}
 /* a1: 0.151698589325
  a2: 0.0610287673771
  a3: 0.00989123992622
  a4: -1.83345043659
  a5: 0.000470165163279
  a6: 1.24445450306
  a7: 0.160839855671*/
