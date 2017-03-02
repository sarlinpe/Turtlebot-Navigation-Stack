#!/usr/bin/env python

# Title:        EE4308 Turtlebot Project
# File:         map_updater.py
# Date:         2017-02-13
# Author:       Preben Jensen Hoel (A0158996B) and Paul-Edouard Sarlin (A0153124U)
# Description:  Kinect processor extracting wall coordinates from the 2D
#               point cloud.


import rospy
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2 as pcl2
from math import cos, sin
import config as cfg


def processPcl(pcl_msg, pose):
    h = pcl_msg.height
    w = pcl_msg.width
    # Isolate ROI from Pcl
    roi = zip(range(w),[int(h/2)]*w)
    pcl = pcl2.read_points(pcl_msg, field_names=("x", "y", "z"),
                           skip_nans=True, uvs=roi)
    # Extract coordinates in world frame
    pcl_global = toGlobalFrame(pcl, pose)
    new_walls = extractWalls(pcl_global)
    return new_walls

# Convert from robot frame to global world frame
def toGlobalFrame(pcl, pose):
    pcl_global = []
    for (y,z,x) in pcl: # curious order...
        y = -y # Y axis is inverted in received message
        X = pose[0] + x*cos(pose[2]) - y*sin(pose[2])
        Y = pose[1] + y*cos(pose[2]) + x*sin(pose[2])
        pcl_global.append((X,Y))
    return pcl_global

# Find walls from point cloud
def extractWalls(pcl):
    candidates = []
    for (x,y) in pcl:
        err_norm_x = abs(x - round(x - .5) - .5) - cfg.WALL_THICKNESS/2
        err_norm_y = abs(y - round(y - .5) - .5) - cfg.WALL_THICKNESS/2
        
        # Check if could be a vertical wall
        if abs(err_norm_x) < cfg.TOL_NORMAL:
            spread_y = y - round(y)
            if abs(spread_y) < cfg.TOL_ALONG:
                x_wall = round(x - 0.5) + 0.5
                y_wall = round(y)
                if (x_wall <= -0.5) or (x_wall >= (cfg.MAP_WIDTH-0.5)) or \
                   (y_wall < 0) or (y_wall >= cfg.MAP_HEIGHT):
                    continue
                candidates = addPoint(candidates, x_wall, y_wall)
    
        # Or a horizontal one
        elif abs(err_norm_y) < cfg.TOL_NORMAL:
            spread_x = x - round(x)
            if abs(spread_x) < cfg.TOL_ALONG:
                x_wall = round(x)
                y_wall = round(y - 0.5) + 0.5
                if (y_wall <= -0.5) or (y_wall >= (cfg.MAP_HEIGHT-0.5)) or \
                   (x_wall < 0) or (x_wall >= cfg.MAP_WIDTH):
                    continue
                candidates = addPoint(candidates, x_wall, y_wall)
    # Filter candidates with enough points
    detected_walls = [(x,y) for (x,y,cnt) in candidates if (cnt >= cfg.TOL_NB_PTS)]
    return detected_walls

# Take into account a new wall
def addPoint(candidates, x_wall, y_wall):
    already_detected = False
    for i in range(len(candidates)):
        if (x_wall, y_wall) == (candidates[i][0], candidates[i][1]):
            candidates[i][2] += 1 # increase number of corresponding points
            already_detected = True
            break
    if not already_detected:
        candidates.append([x_wall, y_wall, 1])
    return candidates
