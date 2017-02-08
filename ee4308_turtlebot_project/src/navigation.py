#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion

from local_planner import LocalPlanner
from global_planner import AStar as pathSearch, globalSmoothing
from map_generator import MapGenerator
import config as cfg


def update(self, odom):
    (v_lin, v_ang) = controller.makePlan(odom)
    cmd = Twist()
    cmd.linear.x = v_lin
    cmd.angular.z = v_ang
    pub.publish(cmd)

    visualisation.publishPath(path)


if __name__ == "__main__":
    global pub, path, controller, visualisation
    rospy.init_node("navigation", anonymous=True)
    rospy.Subscriber("/odom_true", Odometry, update)
    pub = rospy.Publisher("/cmd_vel_mux/input/teleop", Twist, queue_size=1)

    # TODO: check exception if no path is found
    path = pathSearch()
    if cfg.GLOBAL_SMOOTHING:
        path = globalSmoothing(path)
    
    controller = LocalPlanner()

    visualisation = RvizInterface()
    visualisation.publishMap()
    
    try:
        node = LocalPlanner()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Shutting down node: %s", rospy.get_name())
