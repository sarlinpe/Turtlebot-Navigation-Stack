#!/usr/bin/env python

# Title:        EE4308 Turtlebot Project
# File:         local_planner.py
# Date:         2017-02-13
# Author:       Preben Jensen Hoel (A0158996B) and Paul-Edouard Sarlin (A0153124U)
# Description:  Local path planner performing velocity commands computations with
#               path following and smoothing.


import rospy
from math import atan2, sqrt, pow, pi, copysign
import config as cfg


class CtrlStates:
    Orient, Move, Wait = range(3)


class LocalPlanner:
    # Reset the controller, find nearest start point
    def reset(self, path):
        self.ctrl_state = CtrlStates.Move
        self.sum_theta = 0.
        self.sum_dist = 0.
        self.path = path
        self.pts_cnt = 0

    # Compute first target point of the path from current position
    def findNext(self, pose):
        position = (pose[0],pose[1])
        cnt = self.pts_cnt
        dist_best = self.dist(self.path[0], position)
        for i in range(cnt,len(self.path)):
            dist = self.dist(self.path[i], position)
            if dist > dist_best:
                break
            else:
                dist_best = dist
                cnt = i
        if i != (len(self.path)-1):
            angle = atan2(position[1] - self.path[i][1],
                            position[0] - self.path[i][0])
                    - atan2(self.path[i+1][1] - self.path[i][1],
                            self.path[i+1][0] - self.path[i][0])
            #dist_next = self.dist(self.path[i+1], position)
            #dist_inter = self.dist(self.path[i+1], self.path[i])
            #if dist_next < dist_inter:
            if(abs(checkAngle(angle)) < pi/2): # meybe decrease the tolerance
                cnt +=  1
        self.pts_cnt = cnt

    # Update the PI controller, including local smoothing
    def update(self, pose):
        (pos_x, pos_y , theta) = pose

        while True:
            v_lin = 0.
            v_ang = 0.

            if self.ctrl_state == CtrlStates.Wait:
                return (v_lin, v_ang)
            
            self.findNext(pose)
            
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

            if cfg.LOCAL_SMOOTHING:
                (err_dist_tot, err_theta_tot) = self.weighted_errors(err_dist,
                                                                     err_theta)
            else:
                err_theta_tot = err_theta[0]
                err_dist_tot = err_dist[0]

            self.sum_theta += err_theta_tot
            self.sum_dist += err_dist_tot

            # Control intial orentation
            if self.ctrl_state == CtrlStates.Orient :
                # Check if satisfactory => start motion
                if (abs(err_theta_tot) < cfg.TOL_ORIENT) \
                        or (err_dist[0] < cfg.TOL_DIST) :
                    self.ctrl_state = CtrlStates.Move
                    self.sum_theta = 0.
                    self.sum_dist = 0.
                    continue
                # Else adjust orientation
                else:
                    v_ang = cfg.K_P_ORIENT * err_theta_tot
                            + cfg.K_I_ORIENT * self.sum_theta
                    break
            
            # Control motion between waypoints
            elif self.ctrl_state == CtrlStates.Move :
                # Check if satisfactory => next point
                if err_dist[0] < cfg.TOL_DIST :
                    if self.pts_cnt == (len(self.path) - 1):
                        self.ctrl_state = CtrlStates.Wait
                        break
                    else:
                        self.pts_cnt += 1
                        self.sum_theta = 0.
                        self.sum_dist = 0.
                        continue
                else:
                    if (abs(err_theta_tot) > cfg.THR_ORIENT):
                        rospy.loginfo("Orientation error is too big, turning.")
                        self.ctrl_state = CtrlStates.Orient
                        self.sum_theta = 0.
                        self.sum_dist = 0.
                        continue
                    else:
                        v_ang = cfg.K_P_ORIENT * err_theta_tot
                                + cfg.K_I_ORIENT * self.sum_theta
                        v_lin = cfg.K_P_DIST * err_dist_tot
                                + cfg.K_I_DIST * self.sum_dist
                        break
        return self.checkVelocities(v_lin, v_ang)
        
    # Compute weigthed distance and orientation errors in local smoothing
    def weighted_errors(self, err_dist, err_theta):
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
        return (err_dist_tot, err_theta_tot)

    # Euclidian distance
    def dist(self, p1,p2):
        return sqrt(pow(p1[0] - p2[0], 2) + pow(p1[1] - p2[1], 2))
        
    # Enforce velocities limits
    def checkVelocities(self, v_lin, v_ang):
        return (copysign(min(abs(v_lin),cfg.MAX_V_LIN),v_lin),
                copysign(min(abs(v_ang),cfg.MAX_V_ANG), v_ang))

    # Convert the angle to [-pi,pi]
    def checkAngle(self, theta):
        if theta > pi:
            return(theta - 2 * pi)
        if theta < -pi:
            return(theta + 2 * pi)
        else:
            return(theta)
