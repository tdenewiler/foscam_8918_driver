#ifndef FOSCAM_8918_DRIVER_FOSCAM_8918_DRIVER_H
#define FOSCAM_8918_DRIVER_FOSCAM_8918_DRIVER_H

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/videoio/videoio.hpp>

#include <ros/ros.h>
#include <ros/console.h>
#include <ros/time.h>
#include <camera_info_manager/camera_info_manager.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <dynamic_reconfigure/server.h>
#include <foscam_8918_driver/foscam_8918_driverConfig.h>

namespace foscam_8918_driver
{
class Foscam8918
{
 public:
  explicit Foscam8918(ros::NodeHandle nh_);

 private:
  void timerCallback(const ros::TimerEvent &event);

  void configCallback(foscam_8918_driver::foscam_8918_driverConfig &config, uint32_t level);

  void connectToCamera();

  dynamic_reconfigure::Server<foscam_8918_driver::foscam_8918_driverConfig> reconfig_srv_;
  dynamic_reconfigure::Server<foscam_8918_driver::foscam_8918_driverConfig>::CallbackType reconfig_cb_;

  std::string username_;
  std::string password_;
  std::string ip_address_;
  std::string port_;
  std::string url_suffix_;

  int rate_;
  bool have_connection_;
  cv::VideoCapture vcap_;

  boost::shared_ptr<image_transport::ImageTransport> it_;
  image_transport::CameraPublisher image_pub_;
  boost::shared_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;
  cv_bridge::CvImage cv_img_;
};
}

#endif
