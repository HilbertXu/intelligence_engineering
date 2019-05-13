#! /usr/bin/env python
# -*- coding: utf-8 -*-
import rospy 
import os
import sys
from nav_msgs.msg import OccupancyGrid

class map_receiver:
    def __init__(self):
        rospy.init_node("map_receiver")
        rospy.Subscriber("/map", OccupancyGrid, self.mapCallback)
        rospy.spin()
    
    def mapCallback(self, msg):
        print (msg.info.width)
        print (msg.info.height)
        print (msg.info.origin)
        print (len(msg.data))
        # print (msg.info.position)
    
if __name__ == '__main__':
    map = map_receiver()

