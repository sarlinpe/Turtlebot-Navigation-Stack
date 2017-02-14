#!/usr/bin/env python

# Title:        EE4308 Turtlebot Project
# File:         config.py
# Date:         2017-02-13
# Author:       Preben Jensen Hoel (A0158996B) and Paul-Edouard Sarlin (A0153124U)
# Description:  Configuration file containing static parameters used by several modules
#               for map building, path planning, or low-level control.


from math import radians as rad


# MAP PARAMETERS
KNOWN_MAP = False
GOAL_DEFAULT = (4,4)
MAP_WIDTH   = 9
MAP_HEIGHT  = 9
MAP =   [(0.5,0), #begin vertical walls
         (0.5,2),
         (0.5,3),
         (1.5,0),
         (1.5,1),
         (1.5,4),
         (1.5,5),
         (1.5,7),
         (3.5,0),
         (3.5,1),
         (3.5,2),
         (3.5,3),
         (5.5,1),
         (5.5,2),
         (5.5,3),
         (5.5,4),
         (5.5,5),
         (7.5,1),
         (7.5,2),
         (7.5,3),
         (7.5,6),
         (0,6.5), #begin horizontal walls
         (1,3.5),
         (1,6.5),
         (2,5.5),
         (3,5.5),
         (4,5.5),
         (5,5.5),
         (8,0.5),
         (8,6.5)]

X_OFFSET = 0.5
Y_OFFSET = 0.5

RESOLUTION = 0.1
WALL_THICKNESS = 0.2

ORIGIN_X = 0
ORIGIN_Y = 0
ORIGIN_Z = 0


# MAP BUILDING PARAMETERS
TOL_NORMAL = 0.1
TOL_ALONG = 0.20
TOL_NB_PTS = 30


# PATH FINDING PARAMETERS
COST_NORMAL = 1
COST_LOWER = 0.95
COST_MOVE = COST_NORMAL
COST_TURN = COST_LOWER


# SMOOTHING PARAMETERS
LOCAL_SMOOTHING  = True
SMOOTH_NB_PTS   = 4
SMOOTH_WEIGHTS  = [7, 4, 2, 1]

GLOBAL_SMOOTHING = True
SMOOTHING_TOL = 1E-6
ALPHA = 0.2
SMOOTHING_RATE = 1
SMOOTHING_DENSITY = 4


# CONTROL PARAMETERS
TOL_ORIENT  = 0.01
TOL_DIST    = 0.1
THR_ORIENT  = rad(45)
K_P_ORIENT  = 0.9
K_P_DIST    = 0.4
K_I_ORIENT  = 5e-4
K_I_DIST    = 1e-3

MAX_V_LIN = .3
MAX_V_ANG = 2
