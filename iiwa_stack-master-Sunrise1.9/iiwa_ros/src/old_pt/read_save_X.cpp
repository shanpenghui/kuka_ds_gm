#include <fstream>
#include "ros/ros.h"
#include <string>
#include <time.h>
#include <stdio.h>
using namespace std;
int main(int argc,char** argv)
{
 ros::init(argc,argv,"read_save_X");
ros::NodeHandle n;
struct tm *local,*ptr;
time_t t;
t=time(NULL);
ifstream in("/home/edward/ndi/NDI.txt");
local=localtime(&t);
//output<<local->tm_hour<<"\n";
ofstream output("/home/edward/kuka_ws/ndi_data/wrndi_bs_z_1.txt");//，每次运行改名
ros::Rate loop_rate(256);
  while(ros::ok())
  {
  ifstream in("/home/edward/ndi/NDI.txt");
  string ndi;
  getline(in,ndi);
  output<<ndi<<"\n";
  loop_rate.sleep();
   }
  // ROS_INFO("%f", str);
  return 0;
}
