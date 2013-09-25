foscam_8918_driver
======

ROS driver that uses OpenCV to get image stream from Foscam 8918 IP camera and publish with image transport.

To Do
======

 * Read login credentials from a file so that it's not necessary to commit user name and password to VCS.
   * Currently, there is a launch file that is added to .gitignore file based off the example launch file. The ignored launch file should be sort of safe to put login credentials, but it's still feels pretty dangerous.
 * See if there are ways to support pan, tilt, zoom, ROI, speaker, microphone, image resolution, and other features of the Foscam cameras.
