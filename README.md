foscam_8918_driver
======

ROS driver that uses OpenCV to get image stream from Foscam 8918 IP camera and publish with image transport.

To Do
======

 * Read login credentials from a file so that it's not necessary to commit user name and password to VCS.
 * Make this node read and publish camera_info messages appropriately.
 * Add service to set camera_info message from calibration utility.
 * See if there are ways to support pan, tilt, zoom, ROI, speaker, microphone, image resolution, and other features of the Foscam cameras.
