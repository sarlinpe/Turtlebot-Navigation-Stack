#!/usr/bin/env python

"""
    Would be nice to publish the map and the path as rostopics that can be visualized in RViz.
    http://docs.ros.org/api/nav_msgs/html/msg/OccupancyGrid.html
    http://docs.ros.org/api/nav_msgs/html/msg/Path.html
"""

import rospy
from nav_msgs.msg import OccupancyGrid, Path


class MapGenerator:
    def __init__(self):
        self.pub_map = rospy.Publisher("/map", OccupancyGrid, queue_size=1)
        self.pub_path = rospy.Publisher("/path", Path, queue_size=1)
    
        # TODO: get parameters from rosparam server
        self.width = 9
        self.height = 9
        self.resolution = 0.1
        self.wall_thickness = 0.2

        self.map = OccupancyGrid()
        self.map.info.resolution = self.resolution
        self.map.info.width = self.width/self.resolution
        self.map.info.height = self.height/self.resolution
        self.map.info.origin.position.x = 0.
        self.map.info.origin.position.y = 0.
        self.map.info.origin.position.z = 0.
    
        self.path = Path() # May need to set a frame here...

    def publish(self, walls, path):

        # Initialize 2D map with zeros
        map = []
        for i in range(self.map.info.height):
            row = []
            for j in range(self.map.info.width):
                row.append(0)
            map.append(row)

        # Add border by setting pixels to 10

        # Iterate through the walls, set the pixels accordingly

        # Flatten map to self.map.data in a row-major order

        # Construct Path message

        # Publish messages
        self.pub_map.publish(self.map)
        self.pub_path.publish(self.path)
        
