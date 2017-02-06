"""
    Config file for python
"""

#MAP PARAMETERS
KNOWN_MAP = True
MAP_WIDTH   = 9
MAP_HEIGTH  = 9
START   = (0,0)
GOAL    = (4,3)
WALLS = [(0.5,0),
         (0.5,2),
         (0.5,3),
         (1.5,0),
         (1.5,1),
         (3.5,0),
         (3.5,1),
         (3.5,2),
         (3.5,3)]

X_OFFSET = -0.5
Y_OFFSET = -0.5

#
MOVE_COST = 1
TURN_COST = 0.95 

#SMOOTHING PARAMETERS
LOCAL_SMOOTHING = False
GLOBAL_SMOOTHING = True

SMOOTH_NB_PTS   = 4
SMOOTH_WEIGHTS  = [4, 3, 1, 1]

SMOOTHING_TOL = 1E-6
ALPHA = 0.2
SMOOTHING_RATE = 1
SMOOTHING_DENSITY = 4


# CONTROL PARAMETERS
TOL_ORIENT  = 0.01
TOL_DIST    = 0.1
K_P_ORIENT  = 0.8
K_P_DIST    = 0.6
K_I_ORIENT  = 5e-4
K_I_DIST    = 1e-3