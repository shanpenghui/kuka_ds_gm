#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

using namespace sensor_msgs;
using namespace message_filters;

void callback(const ImageConstPtr& image, const CameraInfoConstPtr& cam_info)  //回调中包含多个消息
{
  // Solve all of perception here...
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "get_all_sensor_in_one");

  ros::NodeHandle nh;

  message_filters::Subscriber<Image> image_sub(nh, "image", 1);             // topic1 输入
  message_filters::Subscriber<CameraInfo> info_sub(nh, "camera_info", 1);   // topic2 输入
  TimeSynchronizer<Image, CameraInfo> sync(image_sub, info_sub, 10);       // 同步
  sync.registerCallback(boost::bind(&callback, _1, _2));                   // 回调

  ros::spin();

  return 0;
}