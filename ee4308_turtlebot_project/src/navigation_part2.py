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
from math import cos, sin

from local_planner import LocalPlanner
from global_planner import AStar as pathSearch, globalSmoothing
from map_updater import processPcl
from rviz_interface import RvizInterface
import config as cfg

pose = None


# Update the velocity commands and publish path to RViz
def updateController(odom_msg):
    global pose, init
    cmd = Twist()
    
    pose = extractPose(odom_msg)
    if not init:
        initialisePath()
        init = True

    if cfg.GOAL is None:
        (v_lin, v_ang) = (0,0)
    else:
        (v_lin, v_ang) = controller.update(pose[0], pose[1], pose[2])

    cmd.linear.x = v_lin
    cmd.angular.z = v_ang
    pub.publish(cmd)
    visualisation.publishPath(path)


# Sets a new goal and initialize the path
def newGoal(goal_msg):
    global init
    # Extract goal positionin frame Odom
    X = goal_msg.pose.position.x
    Y = goal_msg.pose.position.y
    # Convert to Gazebo world frame
    x = pose[0] + cfg.X_OFFSET + X*cos(pose[2]) - Y*sin(pose[2])
    y = pose[1] + cfg.Y_OFFSET + Y*cos(pose[2]) + X*sin(pose[2])
    # Check if not out of boundaries
    if (x < 0) or (x >= cfg.MAP_WIDTH) or (y < 0) or (y >= cfg.MAP_HEIGHT):
        cfg.GOAL = None
        rospy.logerr("Goal is out of the working area.")
        return
    # Display
    cfg.GOAL = (int(round(x - cfg.X_OFFSET)),int(round(y - cfg.Y_OFFSET)))
    rospy.loginfo("New goal set: %s", cfg.GOAL)
    initialisePath()
    visualisation.publishPath(path)
    init = False


def updateMap(pcl_msg):
    new_walls = processPcl(pcl_msg, pose)
    if len(new_walls) == 0:
        return
    new_map = cfg.WALLS + [w for w in new_walls if w not in cfg.WALLS]
    # Lock the map and update, or add to a queue that will be pulled by the odom callback.
    # Maybe in a first place try to publish it to Rviz while still using the true map for planning,
    # so that we can make sure that the map building is correct
    # ...


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


def initialisePath():
    global path
    cfg.START = (int(round(pose[0])), int(round(pose[1])))
    rospy.loginfo("Computing path with start %s and goal %s", cfg.START, cfg.GOAL)
    path = pathSearch()
    if cfg.GLOBAL_SMOOTHING:
        path = globalSmoothing(path)
    controller.reset(path, pose)


if __name__ == "__main__":
    global pub, controller, visualisation, init
    rospy.init_node("navigation", anonymous=True)
    rospy.Subscriber("/odom_true", Odometry, updateController)
    rospy.Subscriber("/move_base_simple/goal", PoseStamped, newGoal)
    rospy.Subscriber("/camera/depth/points_throttle", PointCloud2, updateMap)
    pub = rospy.Publisher("/cmd_vel_mux/input/teleop", Twist, queue_size=1)

    controller = LocalPlanner()
    visualisation = RvizInterface()
    visualisation.publishMap()
    init = False
    
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Shutting down node: %s", rospy.get_name())
