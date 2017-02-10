#!/usr/bin/env python

"""
    Code is from http://www.redblobgames.com/pathfinding/a-star/implementation.html
    With adapted data structures (not a real graph)
    
    TODO:
    - create a class that export several search algorithm (A-star, ...)
    - Add exception if goal is outside the defined area
"""

import heapq
import config as cfg


class PriorityQueue:
    def __init__(self):
        self.elements = []
    
    def empty(self):
        return len(self.elements) == 0
    
    def put(self, item, priority):
        heapq.heappush(self.elements, (priority, item))
    
    def get(self):
        return heapq.heappop(self.elements)[1]

def getPt(idx):
    x = idx % cfg.MAP_WIDTH
    y = int(idx / cfg.MAP_WIDTH)
    return (x, y)

def getIdx(pt):
    (x, y) = pt
    return (y * cfg.MAP_WIDTH + x)

def heuristic(a, b):
    (x1, y1) = a
    (x2, y2) = b
    return abs(x1 - x2) + abs(y1 - y2)

def buildPath(came_from):
    path = []
    current = getIdx(cfg.GOAL)
    while current is not None:
        path.insert(0, getPt(current))
        current = came_from[current]
    return path


def AStar():
    frontier = PriorityQueue()
    frontier.put(getIdx((cfg.START)), 0)
    came_from = {}
    cost_so_far = {}
    came_from[getIdx(cfg.START)] = None
    cost_so_far[getIdx(cfg.START)] = 0
    
    while not frontier.empty():
        current = frontier.get()
        
        if current == getIdx(cfg.GOAL):
            return buildPath(came_from)
        
        (x, y) = getPt(current)
        neighbors = [(x + 1, y), (x, y - 1), (x - 1, y), (x, y + 1)]
        rem = []
        for pt in neighbors:       
            (xp, yp) = pt
            if (xp < 0) or (xp >= cfg.MAP_WIDTH) or (yp < 0) or (yp >= cfg.MAP_HEIGTH) \
                    or ((( x + xp) / 2., (y + yp) / 2.) in cfg.WALLS):
                rem.append(pt)
        neighbors = [getIdx(pt) for pt in neighbors if pt not in rem]
        
        for next in neighbors:
            if current != getIdx(cfg.START):
                (x_n, y_n) = getPt(next)
                (x_p, y_p) = getPt(came_from[current])
                if (x_n is not x_p) and (y_n is not y_p):
                    move_cost = cfg.COST_TURN
                else:
                    move_cost = cfg.COST_MOVE
            else:
                move_cost = cfg.COST_MOVE
            new_cost = cost_so_far[current] + move_cost
            if next not in cost_so_far or new_cost < cost_so_far[next]:
                cost_so_far[next] = new_cost
                priority = new_cost + heuristic(cfg.GOAL, getPt(next))
                frontier.put(next, priority)
                came_from[next] = current
    
    raise ValueError('Goal '+str(cfg.GOAL)+'cannot be reached from'+str(cfg.START))


def globalSmoothing(path):
    dense = []
    for i in range(0, len(path) - 1):
        for d in range(0, cfg.SMOOTHING_DENSITY):
            pt = []
            for j in range(0, len(path[0])):
                pt.append(((cfg.SMOOTHING_DENSITY - d) * path[i][j] + d * path[i + 1][j]) / float(cfg.SMOOTHING_DENSITY))
            dense.append(tuple(pt))
    dense.append(path[len(path) - 1])

    smoothed = [list(pt) for pt in dense] # convert from tuple to list
    err = cfg.SMOOTHING_TOL

    while err >= cfg.SMOOTHING_TOL:
        err = 0
        for i in range(1, len(dense) - 1):
            for j in range(0, len(dense[0])):
                tmp = smoothed[i][j]
                smoothed[i][j] = smoothed[i][j] + \
                    cfg.SMOOTHING_RATE * (cfg.ALPHA * (dense[i][j] - smoothed[i][j]) + 
                    (1 - cfg.ALPHA) * (smoothed[i + 1][j] + 
                    smoothed[i - 1][j] - 2. * smoothed[i][j]))
                err = err + abs(tmp - smoothed[i][j])
    return smoothed


if __name__ == "__main__":
    try:
        path = AStar()
        smoothed = globalSmoothing(path)
    except ValueError:
        print("No path found :(")
    else:
        print path
        print smoothed
