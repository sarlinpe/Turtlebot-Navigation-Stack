#!/usr/bin/env python

"""
    See turtlebot_control.cpp in `/doc` for a C++ implementation of PI controller from Lab2
    
    TODO:
    - add the map parameters (wall, height, width) in a ros_param server
    - generate map and path for RViz
    - change this simple node to an action node in order to allow the goal to be sent from RViz
"""

import config as cfg
import rospy
from math import atan2, sqrt, pow, pi
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Vector3
from tf.transformations import euler_from_quaternion

from global_planner import AStar as pathSearch, globalSmoothing
from map_generator import MapGenerator


class CtrlStates:
    Orient, Move, Wait = range(3)


class LocalPlanner:

    def __init__(self):
        rospy.Subscriber("/odom_true", Odometry, self.makePlan)
        self.pub = rospy.Publisher("/cmd_vel_mux/input/teleop", Twist, queue_size=1)
        self.gen = MapGenerator()
        
        # TODO: check exception if no path is found
        self.path = pathSearch()
        if cfg.GLOBAL_SMOOTHING:
            self.path = globalSmoothing(self.path)
        self.gen.publish(cfg.WALLS, self.path)
        self.pts_cnt = 0
        self.ctrl_state = CtrlStates.Orient
        
        self.sum_theta = 0.
        self.sum_dist = 0.


    def makePlan(self, odom):
        # Extract relevant state variable from Odometry message
        quaternion = (
            odom.pose.pose.orientation.x,
            odom.pose.pose.orientation.y,
            odom.pose.pose.orientation.z,
            odom.pose.pose.orientation.w)
        euler = euler_from_quaternion(quaternion)
        theta = euler[2]
        pos_x = odom.pose.pose.position.x - cfg.X_OFFSET
        pos_y = odom.pose.pose.position.y - cfg.Y_OFFSET
        
        v_lin = 0.
        v_ang = 0.
        
        # Control intial orentation
        if self.ctrl_state == CtrlStates.Orient :
            # Compute orientation error
            (x, y) = self.path[self.pts_cnt]
            goal_theta = atan2(y - pos_y, x - pos_x)
            err_theta = self.checkAngle(goal_theta - theta)
            
            # Check if satisfactory => start motion
            if err_theta < cfg.TOL_ORIENT :
                self.ctrl_state = CtrlStates.Move
                self.sum_theta = 0.
                self.sum_dist = 0.
            # Else adjust orientation
            else:
                self.sum_theta += err_theta
                v_ang = cfg.K_P_ORIENT * err_theta + cfg.K_I_ORIENT * self.sum_theta
        
        # Control motion between waypoints
        elif self.ctrl_state == CtrlStates.Move :
            # Check if apply local smoothing or used global one
            if cfg.LOCAL_SMOOTHING:
                nb_err_pts = cfg.SMOOTH_NB_PTS
            else:
                nb_err_pts = 1
            
            # Compute orientation and distance error for the next points
            err_theta = []
            err_dist = []
            for (x, y) in self.path[self.pts_cnt:self.pts_cnt + nb_err_pts]:
                err_theta_raw = atan2(y - pos_y, x - pos_x) - theta
                err_theta.append(self.checkAngle(err_theta_raw))
                err_dist.append(sqrt(pow(x - pos_x, 2) + pow(y - pos_y, 2)))
            
            # TODO:
            # - check if any point is under the tolerance -> shift pts_cnt
            # - computer v_lin and v_ang with a defined PI control law
            
            if err_dist[0] < cfg.TOL_DIST :
                if self.pts_cnt == (len(self.path) - 1):
                    self.ctrl_state = CtrlStates.Wait
                else:
                    self.pts_cnt += 1
                    self.sum_theta = 0. # should we reset the controller or not ?
                    self.sum_dist = 0.
                    # should be try a new iteration with the next points ?
            else:
                # try P-controller first, implement I later
                
                if cfg.LOCAL_SMOOTHING:
                    err_theta_tot = 0
                    err_theta_scaling = 0
                    err_dist_tot = 0
                    err_dist_scaling = 0
                    for i in range(len(err_theta)):
                        err_theta_tot += err_theta[i] * err_dist[i] * cfg.SMOOTH_WEIGHTS[i]
                        err_theta_scaling += err_dist[i] * cfg.SMOOTH_WEIGHTS[i]
                        err_dist_tot += err_dist[i] * cfg.SMOOTH_WEIGHTS[i]
                        err_dist_scaling += cfg.SMOOTH_WEIGHTS[i]
                    err_theta_tot /= err_theta_scaling
                    err_dist_tot /= err_dist_scaling
                    self.sum_theta += err_theta_tot
                    self.sum_dist += err_dist_tot
                else:
                    err_theta_tot = err_theta[0]
                    err_dist_tot = err_dist[0]
                    self.sum_theta += err_theta[0]
                    self.sum_dist += err_dist[0]
                
                v_ang = cfg.K_P_ORIENT * err_theta_tot + K_I_ORIENT * self.sum_theta
                v_lin = cfg.K_P_DIST * err_dist_tot + K_I_DIST * self.sum_dist
        
        # Wait for a new goal point
        else:
            pass
        cmd = Twist()
        cmd.linear.x = v_lin
        cmd.angular.z = v_ang
        self.pub.publish(cmd)


    def checkAngle(self, theta):
        if theta > pi:
            return(theta - 2 * pi)
        if theta < -pi:
            return(theta + 2 * pi)
        else:
            return(theta)


if __name__ == "__main__":
    rospy.init_node("local_planner", anonymous=True)
    try:
        node = LocalPlanner()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Shutting down node: %s", rospy.get_name())

