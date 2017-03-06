#!/usr/bin/env python

# Title:        EE4308 Turtlebot Project
# File:         global_planner.py
# Date:         2017-02-13
# Author:       Preben Jensen Hoel (A0158996B) and Paul-Edouard Sarlin (A0153124U)
# Description:  Global path planner, including path finding, densifying and
#               smoothing.


import heapq
from math import pi, atan2, radians as rad
import config as cfg


# Useful queue class for AStar
class PriorityQueue:
    def __init__(self):
        self.elements = []
    
    def empty(self):
        return len(self.elements) == 0
    
    def put(self, item, priority):
        heapq.heappush(self.elements, (priority, item))
    
    def get(self):
        return heapq.heappop(self.elements)[1]

# Get the coordinates of a node of the graph
def getPt(idx):
    x = idx % cfg.MAP_WIDTH
    y = int(idx / cfg.MAP_WIDTH)
    return (x, y)

# Get the graph index of a grid cell
def getIdx(pt):
    (x, y) = pt
    return (y * cfg.MAP_WIDTH + x)

# Heuristic function used by AStar
def heuristic(a, b):
    (x1, y1) = a
    (x2, y2) = b
    return abs(x1 - x2) + abs(y1 - y2)

# Backtrack from the goal to build the path using the list of nodes
def buildPath(came_from, goal):
    path = []
    current = getIdx(goal)
    while current is not None:
        path.insert(0, getPt(current))
        current = came_from[current]
    return path

# AStar algorithm that finds the shortest path start and goal
def AStar(start, goal, map, theta=0):
    frontier = PriorityQueue()
    frontier.put(getIdx((start)), 0)
    came_from = {}
    cost_so_far = {}
    came_from[getIdx(start)] = None
    cost_so_far[getIdx(start)] = 0
    
    while not frontier.empty():
        current = frontier.get() # Get node with lowest priority
        if current == getIdx(goal):
            return buildPath(came_from, goal)
        
        # Determine reachable nodes
        (x, y) = getPt(current)
        neighbors = [(x + 1, y), (x, y - 1), (x - 1, y), (x, y + 1)]
        rem = []
        for pt in neighbors:       
            (xp, yp) = pt
            if (xp < 0) or (xp >= cfg.MAP_WIDTH) \
                    or (yp < 0) or (yp >= cfg.MAP_HEIGHT) \
                    or ((( x + xp) / 2., (y + yp) / 2.) in map):
                rem.append(pt)
        neighbors = [getIdx(pt) for pt in neighbors if pt not in rem]
        
        for next in neighbors:
            # Checks if turn or straight path, assign corresponding weights
            if current != getIdx(start):
                (x_n, y_n) = getPt(next)
                (x_p, y_p) = getPt(came_from[current])
                if (x_n != x_p) and (y_n != y_p):
                    move_cost = cfg.COST_TURN
                else:
                    move_cost = cfg.COST_MOVE
            else:
                (x_n, y_n) = getPt(next)
                err_theta = checkAngle(atan2(y_n - start[1], x_n - start[0])
                                       - theta)
                if abs(err_theta) < rad(45):
                    move_cost = cfg.COST_LOWER
                else:
                    move_cost = cfg.COST_NORMAL
            # Compute the movement cost from the start (g)
            new_cost = cost_so_far[current] + move_cost
            if next not in cost_so_far or new_cost < cost_so_far[next]:
                cost_so_far[next] = new_cost
                # Compute the total cost from start to goal through this node (f)
                priority = new_cost + heuristic(goal, getPt(next))
                frontier.put(next, priority)
                came_from[next] = current
    raise ValueError('Goal '+str(goal)+' cannot be reached from '+str(start))

# Global smoothing taking into account all the points
def globalSmoothing(path):
    # Densify the path by adding points
    dense = []
    for i in range(0, len(path) - 1):
        for d in range(0, cfg.SMOOTHING_DENSITY):
            pt = []
            for j in range(0, len(path[0])):
                pt.append(((cfg.SMOOTHING_DENSITY - d) * path[i][j] \
                           + d * path[i + 1][j]) / float(cfg.SMOOTHING_DENSITY))
            dense.append(tuple(pt))
    dense.append(path[len(path) - 1])

    smoothed = [list(pt) for pt in dense] # convert from tuple to list
    err = cfg.SMOOTHING_TOL
    
    # Minimizes the cost function using gradient descent
    while err >= cfg.SMOOTHING_TOL:
        err = 0
        for i in range(1, len(dense) - 1):
            for j in range(0, len(dense[0])):
                tmp = smoothed[i][j]
                smoothed[i][j] = smoothed[i][j] + cfg.SMOOTHING_RATE * \
                    (cfg.ALPHA * (dense[i][j] - smoothed[i][j]) +
                     (1 - cfg.ALPHA) * (smoothed[i + 1][j] + smoothed[i - 1][j] -
                                        2. * smoothed[i][j]))
                err = err + abs(tmp - smoothed[i][j])
    return smoothed

def checkAngle(theta):
    if theta > pi:
        return(theta - 2 * pi)
    if theta < -pi:
        return(theta + 2 * pi)
    else:
        return(theta)
