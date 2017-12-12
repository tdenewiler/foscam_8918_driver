#include "foscam_8918_driver/foscam_8918_driver.h"

namespace foscam_8918_driver
{
Foscam8918::Foscam8918(ros::NodeHandle nh_)
    : username_("admin"),
      password_(""),
      ip_address_("192.168.1.1"),
      port_("80"),
      url_suffix_("video.cgi?.mjpg"),
      rate_(10),
      it_(new image_transport::ImageTransport(nh_)),
      image_pub_(it_->advertiseCamera("image_raw", 1)),
      camera_info_manager_(new camera_info_manager::CameraInfoManager(nh_))
{
  reconfig_cb_ = boost::bind(&foscam_8918_driver::Foscam8918::configCallback, this, _1, _2);
  reconfig_srv_.setCallback(reconfig_cb_);

  ros::NodeHandle pnh("~");
  pnh.param("username", username_, username_);
  pnh.param("password", password_, password_);
  pnh.param("ip_address", ip_address_, ip_address_);
  pnh.param("port", port_, port_);
  pnh.param("url_suffix", url_suffix_, url_suffix_);
  pnh.param("rate", rate_, rate_);
  if (rate_ <= 0)
  {
    rate_ = 1;
  }

  timer_ = nh_.createTimer(ros::Duration(1.0 / rate_), &foscam_8918_driver::Foscam8918::timerCallback, this);

  connectToCamera();
}

void Foscam8918::connectToCamera()
{
  const std::string video_stream_address =
      "http://" + username_ + ":" + password_ + "@" + ip_address_ + ":" + port_ + "/" + url_suffix_;

  have_connection_ = true;
  if (!vcap_.open(video_stream_address))
  {
    have_connection_ = false;
    ROS_ERROR("Error opening video stream or file");
  }
}

void Foscam8918::timerCallback(const ros::TimerEvent &event)
{
  if (!have_connection_)
  {
    return;
  }

  cv::Mat image;
  if (!vcap_.read(image))
  {
    ROS_WARN("No frame");
    cv::waitKey();
  }
  cv_img_.header.stamp = ros::Time::now();
  cv_img_.header.frame_id = "foscam";
  cv_img_.encoding = "bgr8";
  cv_img_.image = image;

  sensor_msgs::CameraInfoPtr ci(new sensor_msgs::CameraInfo(camera_info_manager_->getCameraInfo()));
  ci->height = image.rows;
  ci->width = image.cols;
  ci->header = cv_img_.header;

  image_pub_.publish(cv_img_.toImageMsg(), ci);
}

void Foscam8918::configCallback(foscam_8918_driver::foscam_8918_driverConfig &config, uint32_t level)
{
  username_ = config.username;
  password_ = config.password;
  ip_address_ = config.ip_address;
  port_ = config.port;
  url_suffix_ = config.url_suffix;

  if (config.reset_connection)
  {
    config.reset_connection = false;
    connectToCamera();
    ROS_INFO("Resetting connection to camera with current parameters.");
  }
}
}
