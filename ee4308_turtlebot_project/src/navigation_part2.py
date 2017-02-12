#!/usr/bin/env python

"""
    TODO: implement thread-safe communication betzeen the callbacks of odom_msg, goal_msg and pcl_msg
    --> Need to use a lock for pose, cfg.GOAL, and path planning (cfg.MAP when updated ? Or use a queue object...)
"""

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

goal = None
map = []
pose = None

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


# Sets a new goal and initialize the path
def setGoal(goal_local):
    global goal
    with goal_lock:
        goal = goal_local
    rospy.loginfo("New goal set: %s", goal_local)
    computePath()


def updateMap(pcl_msg):
    global map
    with pose_lock:
        pose_local = pose # processPcl might take some time, avoid blocking updateController
    new_walls = processPcl(pcl_msg, pose_local)
    if len(new_walls) == 0:
        return
    new_map = map + [w for w in new_walls if w not in map]
    with map_lock:
        pass
        #map = new_map
    computePath()
    visualisation.publishMap(new_map)


def computePath():
    global path
    with pose_lock:
        start = (int(round(pose[0])), int(round(pose[1])))
    with goal_lock:
        goal_local = goal
    with map_lock:
        #rospy.loginfo("Computing path with start %s and goal %s", cfg.START, cfg.GOAL)
        path_local = pathSearch(start, goal, map)
    if cfg.GLOBAL_SMOOTHING:
        path_local = globalSmoothing(path_local)
    with controller_lock:
        controller.reset(path_local)
    with path_lock:
        path = path_local
    visualisation.publishPath(path_local)
    return path


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
