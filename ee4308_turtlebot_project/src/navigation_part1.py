#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PoseStamped
from tf.transformations import euler_from_quaternion
from math import cos, sin

from local_planner import LocalPlanner
from global_planner import AStar as pathSearch, globalSmoothing
from rviz_interface import RvizInterface
import config as cfg

pose = None

#Update the speed and orientation of the robot and publish path to RViz
def update(odom_msg):
    global pose, init
    cmd = Twist()
    
    pose = extract_pose(odom_msg)
    if not init:
        initialise_path()
        init = True

    if cfg.GOAL is None:
        (v_lin, v_ang) = (0,0)
    else:
        (v_lin, v_ang) = controller.update(pose[0], pose[1], pose[2])

    cmd.linear.x = v_lin
    cmd.angular.z = v_ang
    pub.publish(cmd)
    visualisation.publishPath(path)

# Extract relevant state variable from Odometry message
def extract_pose(odom_msg):   
    quaternion = (odom_msg.pose.pose.orientation.x,
                  odom_msg.pose.pose.orientation.y,
                  odom_msg.pose.pose.orientation.z,
                  odom_msg.pose.pose.orientation.w)
    euler = euler_from_quaternion(quaternion)
    theta = euler[2]
    pos_x = odom_msg.pose.pose.position.x - cfg.X_OFFSET
    pos_y = odom_msg.pose.pose.position.y - cfg.Y_OFFSET
    return (pos_x, pos_y, theta)

#Sets a new goal and initialize the path.
def new_goal(goal_msg):
    global init
    x_g = goal_msg.pose.position.x
    y_g = goal_msg.pose.position.y
    #rospy.loginfo("Received new goal: %s", (x_g,y_g))
    #rospy.loginfo("Position is now: %s", (pose[0],pose[1]))
    x = pose[0] + cfg.X_OFFSET + x_g*cos(pose[2]) - y_g*sin(pose[2])
    y = pose[1] + cfg.Y_OFFSET + y_g*cos(pose[2]) + x_g*sin(pose[2])
    #rospy.loginfo("Absolute goal: %s", (x,y))

    if (x < 0) or (x >= cfg.MAP_WIDTH) or (y < 0) or (y >= cfg.MAP_HEIGHT):
        cfg.GOAL = None
        rospy.logerr("Goal is out of the working area.")
        return
    
    cfg.GOAL = (int(round(x - cfg.X_OFFSET)),int(round(y - cfg.Y_OFFSET)))
    rospy.loginfo("New goal set: %s", cfg.GOAL)
    initialise_path()
    visualisation.publishPath(path)
    init = False


def initialise_path():
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
    rospy.Subscriber("/odom_true", Odometry, update)
    rospy.Subscriber("/move_base_simple/goal", PoseStamped, new_goal)
    pub = rospy.Publisher("/cmd_vel_mux/input/teleop", Twist, queue_size=1)

    controller = LocalPlanner()
    visualisation = RvizInterface()
    visualisation.publishMap()
    init = False
    
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Shutting down node: %s", rospy.get_name())
