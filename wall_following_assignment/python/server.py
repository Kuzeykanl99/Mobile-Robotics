#!/usr/bin/env python

import rospy

from dynamic_reconfigure.server import Server
from wall_following_assignment.cfg import WallConfig

def callback(config, level):
    return config

if __name__ == "__main__":
    rospy.init_node("wall_following_assignment", anonymous = False)

    srv = Server(WallConfig, callback)
    rospy.spin()
