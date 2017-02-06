#!/usr/bin/env python

"""
    Would be nice to publish the map and the path as rostopics that can be visualized in RViz.
    http://docs.ros.org/api/nav_msgs/html/msg/OccupancyGrid.html
    http://docs.ros.org/api/nav_msgs/html/msg/Path.html
"""

import rospy
from nav_msgs.msg import OccupancyGrid, Path, Pose
from math import floor


class MapGenerator:
    def __init__(self):
        self.pub_map = rospy.Publisher("/map", OccupancyGrid, queue_size=1)
        self.pub_path = rospy.Publisher("/path", Path, queue_size=1)
    
        # TODO: get parameters from rosparam server
        self.width = 6
        self.height = 6
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
        offset_x = 0.5
        offset_y = 0.5

        # Initialize 2D map with zeros
        map = []
        for i in range(self.map.info.height):
            row = []
            for j in range(self.map.info.width):
                row.append(0)
            map.append(row)

        # Add border by setting pixels to 100
        for i in range(self.map.info.height):
            map[i][0] = 100
            map[i][self.map.info.width-1] = 100
        for j in range(self.map.info.width):
            map[0][j] = 100
            map[self.map.info.height-1][j] = 100

        # Iterate through the walls, set the pixels accordingly
        for (x,y) in walls:
            if (y % 1) == 0:
                # Wall is vertical
                x += offset_x
                y += offset_y
                for i in range(1/self.resolution):
                    map[x/self.resoltion][(y-0.5)/self.resolution+i] = 100
                    map[x/self.resoltion-1][(y-0.5)/self.resolution+i] = 100

            else:
                # Wall is horizontal
                x += offset_x
                y += offset_y
                for i in range(1/self.resolution):
                    map[(x-0.5)/self.resolution+i][y/self.resoltion] = 100
                    map[(x-0.5)/self.resolution+i][y/self.resoltion-1] = 100

        # Flatten map to self.map.data in a row-major order
        self.map.data = sum(map,[])

        # Construct Path message
        salf.path.pose[:] = []
        for i in range(len(path)):
            p = Pose()
            
            
            self.path.pose.append(p)
        

        # Publish messages
        self.pub_map.publish(self.map)
        self.pub_path.publish(self.path)
        print map

if __name__ == "__main__":
    width   = 9
    height  = 9
    start   = (0,0)
    goal    = (4,4)
    walls = [(0.5,0),
             (0.5,2),
             (0.5,3),
             (1.5,0),
             (1.5,1),
             (3.5,0),
             (3.5,1),
             (3.5,2),
             (3.5,3),
             (3.5,4)]
    path = [(0,0),
            (0,1),
            (1,1),
            (1,2),
            (2,2),
            (3,2),
            (3,3),
            (3,4),
            (3,5),
            (4,5),
            (4,4)]
    m = MapGenerator()
    m.publish(walls,path)
