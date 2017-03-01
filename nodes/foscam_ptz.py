#!/usr/bin/python
# -*- coding: utf-8 -*-
"""
Node that will subscribe to a commanded velocity and move a Foscam PTZ camera.
"""

import urllib
from enum import Enum
import rospy
from geometry_msgs.msg import Twist

#pylint: disable=no-init
# pylint: disable=too-few-public-methods
class Commands(Enum):
    """
    Available PTZ commands.
    """
    move_up = 0
    stop_up = 1
    move_down = 2
    stop_down = 3
    move_left = 4
    stop_left = 5
    move_right = 6
    stop_right = 7
# pylint: enable=too-few-public-methods
# pylint: enable=no-init

class FoscamPTZ(object):
    """
    Get velocities and move camera.
    """
    def __init__(self):
        self.username = rospy.get_param('~username', 'admin')
        self.password = rospy.get_param('~password', '')
        self.ip_address = rospy.get_param('~ip_address', '192.168.1.1')
        self.port = rospy.get_param('~port', '80')
        self.stopped = False

        rospy.Subscriber('twist', Twist, self.twist_cb)

        rospy.spin()

    def twist_cb(self, msg):
        """
        Convert desired velocities into motion commands.
        """
# pylint: disable=no-member
        # Don't bother sending commands if all zeroes are requested.
        if msg.linear.x == 0.0 and msg.angular.z == 0:
            if self.stopped:
                return
            direction = Commands.stop_up.value
            self.move(direction)
            direction = Commands.stop_left.value
            self.move(direction)
            self.stopped = True
            return

        if msg.linear.x > 0:
            direction = Commands.move_up.value
        elif msg.linear.x < 0:
            direction = Commands.move_down.value
        else:
            direction = Commands.stop_down.value

        self.move(direction)

        if msg.angular.z > 0:
            direction = Commands.move_right.value
        elif msg.angular.z < 0:
            direction = Commands.move_left.value
        else:
            direction = Commands.stop_left.value

        self.move(direction)
        self.stopped = False
# pylint: enable=no-member

    def move(self, direction):
        """
        Create command to move camera in desired direction.
        """
        cmd = {'command':direction}
        self.send_cmd('decoder_control.cgi', cmd)

    def send_cmd(self, cgi, parameters):
        """
        Send command to camera.
        """
        url = 'http://%s:%s/%s?user=%s&pwd=%s' % (self.ip_address,
                                                  self.port,
                                                  cgi,
                                                  self.username,
                                                  self.password)
        for param in parameters:
            url = url + '&%s=%s' % (param, parameters[param])

        return urllib.urlopen(url)

if __name__ == '__main__':
    rospy.init_node('foscam_ptz')
    try:
        FoscamPTZ()
    except rospy.ROSInterruptException:
        pass
