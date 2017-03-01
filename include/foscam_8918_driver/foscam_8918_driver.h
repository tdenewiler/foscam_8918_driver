#ifndef FOSCAM_DRIVER_H
#define FOSCAM_DRIVER_H

// System includes.
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <iostream>

// OpenCV includes.
#include <opencv2/opencv.hpp>

// ROS includes.
#include <ros/ros.h>
#include <ros/console.h>
#include <ros/time.h>
#include <camera_info_manager/camera_info_manager.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

// Dynamic reconfigure.
#include <dynamic_reconfigure/server.h>
#include <foscam_8918_driver/foscam_8918_driverConfig.h>

namespace foscam_8918_driver
{
class Foscam8918
{
public:
  //! Constructor.
  //! \param nh_ The node handle that topics and parameters are attached to.
  explicit Foscam8918(ros::NodeHandle nh_);

  //! Destructor.
  ~Foscam8918();

private:
  //! Callback function for timer that kicks off all the work.
  void timerCallback(const ros::TimerEvent& event);

  //! Callback function for dynamic reconfigure server.
  void configCallback(foscam_8918_driver::foscam_8918_driverConfig &config, uint32_t level);

  //! Connect to the camera.
  bool connectToCamera();

  //! Dynamic reconfigure server.
  dynamic_reconfigure::Server<foscam_8918_driver::foscam_8918_driverConfig> reconfig_srv_;
  //! Dynamic reconfigure callback function.
  dynamic_reconfigure::Server<foscam_8918_driver::foscam_8918_driverConfig>::CallbackType reconfig_cb_;

  //! Parameters needed to connect to camera.
  std::string username_;
  std::string password_;
  std::string ip_address_;
  std::string port_;
  std::string url_suffix_;

  //! Variables for capturing video.
  int rate_;
  bool have_connection_;
  cv::VideoCapture vcap_;

  //! Publishing camera data.
  boost::shared_ptr<image_transport::ImageTransport> it_;
  image_transport::CameraPublisher image_pub_;
  boost::shared_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;
  cv_bridge::CvImage cv_img_;
};
}

#endif // FOSCAM_DRIVER_H
