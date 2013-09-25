#include "foscam_8918_driver/foscam_8918_driver.h"

int main(int argc, char **argv)
{
    // Set up ROS.
    ros::init(argc, argv, "foscam_8918_driver");
    ros::NodeHandle nh;

    Foscam8918Driver::Foscam8918 foscam_8918(nh);

    return 0;
}
