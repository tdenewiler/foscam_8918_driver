#include "foscam_8918_driver/foscam_8918_driver.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "foscam_8918_driver");
  ros::NodeHandle nh;

  foscam_8918_driver::Foscam8918 node(nh);

  ros::spin();

  return 0;
}
