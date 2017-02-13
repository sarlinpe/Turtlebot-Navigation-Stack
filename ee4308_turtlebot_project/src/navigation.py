#!/usr/bin/env python

# Title:        EE4308 Turtlebot Project
# File:         navigation.py
# Date:         2017-02-13
# Author:       Preben Jensen Hoel (A0158996B) and Paul-Edouard Sarlin (A0153124U)
# Description:  Main node of the navigation scheme. Handles communication with other ROS
#               components and manages computational units such as local and global
#               planners or Kinect processor.


import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import PointCloud2
from tf.transformations import euler_from_quaternion
from threading import Lock
from math import cos, sin

from local_planner import LocalPlanner
from global_planner import AStar as pathSearch, globalSmoothing
from map_updater import processPcl
from rviz_interface import RvizInterface
import config as cfg


path_raw = None
path = None
goal = None
map_updated = []
pose = None
ERROR = False

# In rospy callbacks can be called in different threads
controller_lock = Lock()
pose_lock = Lock()
map_lock = Lock()
goal_lock = Lock()
path_lock = Lock()

# Update the velocity commands and publish path to RViz
def updateController(odom_msg):
    global pose, init
    cmd = Twist()
    
    with pose_lock:
        pose = extractPose(odom_msg)
    if ERROR:
        (v_lin, v_ang) = (0,0)
        rospy.loginfo("Stopping robot.")
    else:
        if not init:
            setGoal(cfg.GOAL_DEFAULT)
            init = True
        with controller_lock:
            (v_lin, v_ang) = controller.update(pose)
    cmd.linear.x = v_lin
    cmd.angular.z = v_ang
    pub.publish(cmd)
    with path_lock:
        visualisation.publishPath(path)


# Wrapper as a callback
def newGoal(goal_msg):
    # Extract goal positionin frame Odom
    X = goal_msg.pose.position.x
    Y = goal_msg.pose.position.y
    # Convert to Gazebo world frame
    with pose_lock:
        x = pose[0] + cfg.X_OFFSET + X*cos(pose[2]) - Y*sin(pose[2])
        y = pose[1] + cfg.Y_OFFSET + Y*cos(pose[2]) + X*sin(pose[2])
    if (x < 0) or (x >= cfg.MAP_WIDTH) or (y < 0) or (y >= cfg.MAP_HEIGHT):
        rospy.logerr("Goal is out of the working area.")
        return
    setGoal((int(round(x - cfg.X_OFFSET)),int(round(y - cfg.Y_OFFSET))))
    with path_lock:    
        visualisation.publishPath(path)


# Sets a new goal and initialize the path
def setGoal(goal_local):
    global goal
    with goal_lock:
        goal = goal_local
    rospy.loginfo("New goal set: %s", goal_local)
    computePath()


def updateMap(pcl_msg):
    global map_updated
    #rospy.loginfo("Process PointCloud.")
    with pose_lock:
        pose_local = pose # processPcl might take some time, avoid blocking updateController
    detected_walls = processPcl(pcl_msg, pose_local)
    new_walls = [w for w in detected_walls if w not in map_updated]
    if len(new_walls) == 0:
        return
    rospy.loginfo("Discovered new walls: %s", new_walls)
    new_map = map_updated + new_walls
    with map_lock:
        map_updated = new_map
    if not cfg.KNOWN_MAP:
        rospy.loginfo("Map updated, compute new path.")
        computePath()
    visualisation.publishMap(new_map)


def computePath():
    global path, path_raw, ERROR
    # Create local copies for path, pose, goal
    with path_lock:
        path_astar_last = path_raw
    with pose_lock:
        start = (int(round(pose[0])), int(round(pose[1])))
        theta = pose[2]
    with goal_lock:
        goal_local = goal
    # Compute AStar and smoothed paths
    try:
        if cfg.KNOWN_MAP:
            path_astar = pathSearch(start, goal_local, cfg.MAP, theta)
        else:
            with map_lock:
                path_astar = pathSearch(start, goal_local, map_updated, theta)
    except ValueError as err:
        ERROR = True
        rospy.logerr("%s", err)
        return
    else:
        ERROR = False
    if cfg.GLOBAL_SMOOTHING:
        path_final = globalSmoothing(path_astar)
    else:
        path_final = path_astar
    # Update if path changed
    if (path_astar_last is None) or (path_astar[-1] != path_astar_last[-1]) or \
       (not (set(path_astar) <= set(path_astar_last))) :
        with path_lock:
            path = path_final
            path_raw = path_astar
        with controller_lock:
            controller.reset(path_final)
        rospy.loginfo("Reset controller with new path.")
    else:
        rospy.loginfo("Keep same path, no obstacle on path.")

        

def extractPose(odom_msg):
    # Extract relevant state variable from Odometry message
    quaternion = (odom_msg.pose.pose.orientation.x,
                  odom_msg.pose.pose.orientation.y,
                  odom_msg.pose.pose.orientation.z,
                  odom_msg.pose.pose.orientation.w)
    euler = euler_from_quaternion(quaternion)
    theta = euler[2]
    pos_x = odom_msg.pose.pose.position.x - cfg.X_OFFSET
    pos_y = odom_msg.pose.pose.position.y - cfg.Y_OFFSET
    return (pos_x, pos_y, theta)


if __name__ == "__main__":
    global pub, controller, visualisation, init
    rospy.init_node("navigation", anonymous=True)
    rospy.Subscriber("/odom_true", Odometry, updateController)
    rospy.Subscriber("/move_base_simple/goal", PoseStamped, newGoal)
    rospy.Subscriber("/camera/depth/points_throttle", PointCloud2, updateMap)
    pub = rospy.Publisher("/cmd_vel_mux/input/teleop", Twist, queue_size=1)

    controller = LocalPlanner()
    visualisation = RvizInterface()
    if cfg.KNOWN_MAP:
        visualisation.publishMap(cfg.MAP)
    init = False
    
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Shutting down node: %s", rospy.get_name())
