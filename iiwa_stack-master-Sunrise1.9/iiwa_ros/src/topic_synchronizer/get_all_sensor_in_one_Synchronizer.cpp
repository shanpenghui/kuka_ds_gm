#include <algorithm>
#include <chrono>
#include "ros/ros.h"
#include <sstream>
#include <iostream>
#include <fstream>
#include <string>
#include <stdio.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
//#include <message_filters/sync_policies/exact_time.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/WrenchStamped.h>       
#include <geometry_msgs/PoseArray.h>  
#include "std_msgs/Float64.h"
#include <rosbag/bag.h>
#include "ctime"
#include "time.h"
#include "System.h"
#include "Converter.h"
#include "PangolinViewer.h"
#include "Viewer.h"
#include <boost/foreach.hpp>

using namespace std;
rosbag::Bag bag_record;
#if 1
string int2string(int value)
{
    stringstream ss;
    ss<<value;
    return ss.str();
}
#endif

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}

    void GrabRGBD_IMU(const sensor_msgs::ImageConstPtr& kinect2color_msg, const sensor_msgs::ImageConstPtr& kinect2depth_msg, const sensor_msgs::ImuConstPtr& imu_msg);

    ORB_SLAM2::System* mpSLAM;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "message_filter_node");
  ros::Time::init();
  ros::start();
  string orbVocFile = "/home/robooster/viorb_config/config/ORBvoc.bin";
  string orbSetiingsFile = "/home/robooster/viorb_config/config/kinect2_sd.yaml";

  #if 1
  ORB_SLAM2::Viewer* viewer;

  viewer = new ORB_SLAM2::PangolinViewer(orbSetiingsFile);
  ORB_SLAM2::System orbslam( orbVocFile, orbSetiingsFile ,ORB_SLAM2::System::RGBD, viewer );
  #endif

  ImageGrabber igb(&orbslam);

  ros::NodeHandle nh;

  #if 1
  ROS_INFO("start message filter");
  time_t t=std::time(0);
  struct tm * now = std::localtime( & t );
  string file_name;
  //the name of bag file is better to be determined by the system time
  file_name=int2string(now->tm_year + 1900)+
          '-'+int2string(now->tm_mon + 1)+
          '-'+int2string(now->tm_mday)+
          '-'+int2string(now->tm_hour)+
          '-'+int2string(now->tm_min)+
          '-'+int2string(now->tm_sec)+
            ".bag";
  bag_record.open(file_name, rosbag::bagmode::Write);
  #endif

  message_filters::Subscriber<sensor_msgs::Image> kinect2color_sub(nh, "/kinect2/qhd/image_color_rect", 1);
  message_filters::Subscriber<sensor_msgs::Image> kinect2depth_sub(nh, "/kinect2/qhd/image_depth_rect", 1);
  message_filters::Subscriber<sensor_msgs::Imu> imu_sub(nh, "/imu_os3dm/imu_raw", 1);//订阅imu的Topic
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Imu> MySyncPolicy;
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(20), kinect2color_sub, kinect2depth_sub, imu_sub);
  sync.registerCallback(boost::bind(&ImageGrabber::GrabRGBD_IMU, &igb, _1, _2, _3));
  ros::spin();
  bag_record.close();
  orbslam.Shutdown();
  ros::shutdown();
  return 0;
}

void ImageGrabber::GrabRGBD_IMU(const sensor_msgs::ImageConstPtr& kinect2color_msg, const sensor_msgs::ImageConstPtr& kinect2depth_msg, const sensor_msgs::ImuConstPtr& imu_msg)
{
    bag_record.write("/kinect2/qhd/image_color_rect", kinect2color_msg->header.stamp.now(), kinect2color_msg);
    bag_record.write("/kinect2/qhd/image_depth_rect", kinect2depth_msg->header.stamp.now(), kinect2depth_msg);
    bag_record.write("/imu_os3dm/imu_raw", imu_msg->header.stamp.now(), imu_msg);

    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrRGB;
    try
    {
        cv_ptrRGB = cv_bridge::toCvShare(kinect2color_msg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrD;
    try
    {
        cv_ptrD = cv_bridge::toCvShare(kinect2depth_msg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    mpSLAM->TrackRGBD(cv_ptrRGB->image,cv_ptrD->image,cv_ptrRGB->header.stamp.toSec());
}
--------------------- 
作者：all3n531 
来源：CSDN 
原文：https://blog.csdn.net/wheelfjl/article/details/78653321 
版权声明：本文为博主原创文章，转载请附上博文链接！