#!/usr/bin/env python

# Title:        EE4308 Turtlebot Project
# File:         rviz_interface.py
# Date:         2017-02-13
# Author:       Preben Jensen Hoel (A0158996B) and Paul-Edouard Sarlin (A0153124U)
# Description:  Publishes the map and path as standard ROS messages to be
#               displayed in Rviz.


import rospy
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
from math import floor
import config as cfg


class RvizInterface:
    def __init__(self):
        self.pub_map = rospy.Publisher("/map", OccupancyGrid, queue_size=1,
                                       latch=True)
        self.pub_path = rospy.Publisher("/path", Path, queue_size=1, latch=True)
    
        self.map = OccupancyGrid()
        self.map.header.frame_id = "world"
        self.map.info.resolution = cfg.RESOLUTION
        self.map.info.width = int(cfg.MAP_WIDTH / cfg.RESOLUTION)
        self.map.info.height = int(cfg.MAP_HEIGHT / cfg.RESOLUTION)
        self.map.info.origin.position.x = cfg.ORIGIN_X
        self.map.info.origin.position.y = cfg.ORIGIN_Y
        self.map.info.origin.position.z = cfg.ORIGIN_Z
    
        self.path = Path()
        self.path.header.frame_id = "world"

    # Contruct and publish the path message
    def publishPath(self, path):
        self.path.poses[:] = []
        for i in range(len(path)):
            p = PoseStamped()
            p.pose.position.x = path[i][0] + cfg.X_OFFSET
            p.pose.position.y = path[i][1] + cfg.Y_OFFSET
            p.pose.position.z = 0
            self.path.poses.append(p)
        self.pub_path.publish(self.path)
    
    # Contruct and publish the map message (Occupancy Grid)
    def publishMap(self, walls):
            # Initialize 2D map with zeros
            map = []
            for i in range(self.map.info.height):
                row = []
                for j in range(self.map.info.width):
                    row.append(0)
                map.append(row)

            # Add border
            for i in range(self.map.info.height):
                map[i][0] = 100
                map[i][self.map.info.width - 1] = 100
            for j in range(self.map.info.width):
                map[0][j] = 100
                map[self.map.info.height - 1][j] = 100

            # Iterate through the walls, set the pixels accordingly
            for (x, y) in walls:
                if (y % 1) == 0:
                    # Wall is vertical
                    y += cfg.Y_OFFSET
                    x += cfg.X_OFFSET
                    for i in range(int(1 / cfg.RESOLUTION)):
                        map[int(x / cfg.RESOLUTION)] \
                            [int((y - cfg.Y_OFFSET) / cfg.RESOLUTION + i)] = 100
                        map[int(x / cfg.RESOLUTION - 1)] \
                            [int((y - cfg.Y_OFFSET) / cfg.RESOLUTION + i)] = 100
                else:
                    # Wall is horizontal
                    y += cfg.Y_OFFSET
                    x += cfg.X_OFFSET
                    for i in range(int(1 / cfg.RESOLUTION)):
                        map[int((x - cfg.X_OFFSET) / cfg.RESOLUTION + i)] \
                            [int(y / cfg.RESOLUTION)] = 100
                        map[int((x - cfg.X_OFFSET) / cfg.RESOLUTION + i)] \
                            [int(y / cfg.RESOLUTION - 1)] = 100

            self.map.data = []
            # Flatten map to self.map.data in a row-major order, publish
            for i in range(len(map)):
                for j in range(len(map[0])):
                    self.map.data.append(map[j][i])
            self.pub_map.publish(self.map)

