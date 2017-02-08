#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PosedStamped
from tf.transformations import euler_from_quaternion

from local_planner import LocalPlanner
from global_planner import AStar as pathSearch, globalSmoothing
from map_generator import MapGenerator
import config as cfg

pose = None


def update(self, odom_msg):
    global pose
    
    cmd = Twist()
    
    if cfg.GOAL is None:
        (v_lin, v_ang) = (0,0)
    else:
        # Extract relevant state variable from Odometry message
        quaternion = (odom.pose.pose.orientation.x,
                      odom.pose.pose.orientation.y,
                      odom.pose.pose.orientation.z,
                      odom.pose.pose.orientation.w)
        euler = euler_from_quaternion(quaternion)
        theta = euler[2]
        pos_x = odom.pose.pose.position.x - cfg.X_OFFSET
        pos_y = odom.pose.pose.position.y - cfg.Y_OFFSET
        pose = (pos_x, pos_y, theta)
        (v_lin, v_ang) = controller.update(pose_x, pose_y, theta)

    cmd.linear.x = v_lin
    cmd.angular.z = v_ang
    pub.publish(cmd)

    visualisation.publishPath(path)


def new_goal(slef, goal_msg):
    x = goal_msg.pose.position.x  - cfg.X_OFFSET
    y = goal_msg.pose.position.y  - cfg.Y_OFFSET
    
    if (x < 0) or (x >= cfg.MAP_WIDTH) or (y < 0) or (y >= cfg.MAP_HEIGTH):
        cfg.GOAL = None
        rospy.logerr("Goal is out of the working area.")
        return
    
    x = goal_msg.pose.position.x  - cfg.X_OFFSET
    y = goal_msg.pose.position.y  - cfg.Y_OFFSET
    cfg.GOAL = (int(round(x)),int(rount(y)))
    cfg.START = (pose[0], pose[1])
    initialise_path()


def initialise_path():
    path = pathSearch()
    if cfg.GLOBAL_SMOOTHING:
        path = globalSmoothing(path)
    controller.reset(path)


if __name__ == "__main__":
    global pub, path, controller, visualisation
    rospy.init_node("navigation", anonymous=True)
    rospy.Subscriber("/odom_true", Odometry, update)
    rospy.Subscriber("/move_base_simple/goal", PoseStamped, new_goal)
    pub = rospy.Publisher("/cmd_vel_mux/input/teleop", Twist, queue_size=1)

    controller = LocalPlanner()
    visualisation = RvizInterface()
    visualisation.publishMap()
    
    initialise_path()
    
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Shutting down node: %s", rospy.get_name())
