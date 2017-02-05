#!/usr/bin/env python

"""
    Code is from http://www.redblobgames.com/pathfinding/a-star/implementation.html
    With adapted data structures (not a real graph)
    
    TODO:
    - create a class that export several search algorithm (A-star, ...)
"""

import heapq

height  = None
width   = None

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
    x = idx % width
    y = idx / width
    return (x, y)

def getIdx(pt):
    (x,y) = pt
    return (y * width + x)

def heuristic(a, b):
    (x1, y1) = a
    (x2, y2) = b
    return abs(x1 - x2) + abs(y1 - y2)

def buildPath(came_from, goal):
    path = [goal]
    current = getIdx(goal)
    while current:
        current = came_from[current]
        path.insert(0, getPt(current))
    return path

            
def AStar(start, goal, walls, w, h):
    global width, height
    width = w
    height = h
    
    frontier = PriorityQueue()
    frontier.put(getIdx(start), 0)
    came_from = {}
    cost_so_far = {}
    came_from[getIdx(start)] = None
    cost_so_far[getIdx(start)] = 0
    
    while not frontier.empty():
        current = frontier.get()
        
        if current == getIdx(goal):
            #return came_from, cost_so_far
            return buildPath(came_from, goal)
        
        (x,y) = getPt(current)
        neighbors = [(x+1,y), (x,y-1), (x-1,y), (x,y+1)]
        for pt in neighbors:
            (xp,yp) = pt
            if (xp < 0) or (xp >= width) or (yp < 0) or (yp >= height) \
                        or (((x+xp)/2.,(y+yp)/2.) in walls):
                neighbors.remove(pt)
                continue
        neighbors = [getIdx(p) for p in neighbors]
        
        for next in neighbors:
            new_cost = cost_so_far[current] + 1 # can be changed later on
            if next not in cost_so_far or new_cost < cost_so_far[next]:
                cost_so_far[next] = new_cost
                priority = new_cost + heuristic(goal, getPt(next))
                frontier.put(next, priority)
                came_from[next] = current
    
    raise ValueError('Goal cannot be reached')

def globalSmoothing(path, alpha):
    rate = 1
    tol = 1e-6

    smoothed = [list(pt) for pt in path] # convert from tuple to list
    err = tol

    while err >= tol:
        err = 0
        for i in range(1,len(path)-1):
            for j in range(0,len(path[0])):
                tmp = smoothed[i][j]
                smoothed[i][j] = smoothed[i][j] + \
                                 rate*(alpha*(path[i][j]-smoothed[i][j]) + \
                                       (1-alpha)*(smoothed[i+1][j]+smoothed[i-1][j]-2.*smoothed[i][j]))
                err = err + abs(tmp - smoothed[i][j])
    return smoothed

if __name__ == "__main__":
    
    start = (0,0)
    goal = (4,4)
    walls = [(0.5,0),
             (0.5,2),
             (0.5,3),
             (1.5,0),
             (1.5,1),
             (3.5,0),
             (3.5,1),
             (3.5,2),
             (3.5,3),
             (3.5,4)]

    try:
        path = AStar(start, goal, walls, 9, 9)
        smoothed = globalSmoothing(path,0.7)
    except ValueError:
        print("No path found :(")
    else:
        print path
        print smoothed

