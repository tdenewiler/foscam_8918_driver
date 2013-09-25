#include "foscam_8918_driver/foscam_8918_driver.h"

Foscam8918Driver::Foscam8918::Foscam8918(ros::NodeHandle nh_) :
    it_(new image_transport::ImageTransport(nh_)),
    image_pub_(it_->advertiseCamera("image_raw", 1)),
    camera_info_manager_(new camera_info_manager::CameraInfoManager(nh_))
{
    // Set up a dynamic reconfigure server.
    reconfig_cb = boost::bind(&Foscam8918Driver::Foscam8918::configCallback, this, _1, _2);
    reconfig_srv.setCallback(reconfig_cb);

    // Initialize node parameters.
    ros::NodeHandle pnh("~");
    pnh.param("username",   username_,   std::string("change_me"));
    pnh.param("password",   password_,   std::string("change_me"));
    pnh.param("ip_address", ip_address_, std::string("192.168.1.1"));
    pnh.param("port",       port_,       std::string("80"));
    pnh.param("url_suffix", url_suffix_, std::string("video.cgi?.mjpg"));
    pnh.param("debug_show_image_window", debug_show_image_window_, false);
    pnh.param("rate",       rate_,       int(10));
    if (rate_ <= 0)
    {
        rate_ = 1;
    }

    // Create a timer callback.
    ros::Timer timer = nh_.createTimer(ros::Duration(1.0/rate_), &Foscam8918Driver::Foscam8918::timerCallback, this);

    // Connect to the camera.
    connectToCamera();

    // Let callbacks take over.
    ros::spin();
}

Foscam8918Driver::Foscam8918::~Foscam8918()
{
}

bool Foscam8918Driver::Foscam8918::connectToCamera()
{
    // URL of camera video stream.
    const std::string video_stream_address = "http://" + username_ + ":" + password_ + "@" + ip_address_ + ":" + port_ + "/" + url_suffix_;

    // Open the video stream and make sure it's opened.
    have_connection_ = true;
    if (!vcap_.open(video_stream_address))
    {
        have_connection_ = false;
        ROS_ERROR("Error opening video stream or file");
        return false;
    }

    return true;
}

void Foscam8918Driver::Foscam8918::timerCallback(const ros::TimerEvent& event)
{
    cv::Mat image;

    if (have_connection_)
    {
        if (!vcap_.read(image))
        {
            ROS_WARN("No frame");
            cv::waitKey();
        }
        //! \todo Clean up the header values with use of parameters and better spot for getting time.
        cv_img_.header.stamp = ros::Time::now();
        cv_img_.header.frame_id = "foscam";
        cv_img_.encoding = "bgr8";
        cv_img_.image = image;

        sensor_msgs::CameraInfoPtr ci(new sensor_msgs::CameraInfo(camera_info_manager_->getCameraInfo()));
        ci->height = image.rows;
        ci->width = image.cols;
        ci->header = cv_img_.header;

        image_pub_.publish(cv_img_.toImageMsg(), ci);

        if (debug_show_image_window_)
        {
            cv::imshow("Output Window", image);
            cv::waitKey(1);
        }
    }
}

void Foscam8918Driver::Foscam8918::configCallback(foscam_8918_driver::foscam_8918_driverConfig &config, uint32_t level)
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
