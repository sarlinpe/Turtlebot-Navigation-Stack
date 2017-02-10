#!/usr/bin/env python

WALL_THICKNESS = 0.2
TOL_NORMAL = 0.05
TOL_ALONG = 0.35
TOL_NB_PTS = 2

def extractWalls(pcl):
    candidates = []
    
    for (x,y,z) in pcl:
        err_norm_x = abs(x - round(x - .5) - .5) - WALL_THICKNESS/2
        err_norm_y = abs(y - round(y - .5) - .5) - WALL_THICKNESS/2

        # Check if could be a vertical wall
        if abs(err_norm_x) < TOL_NORMAL:
            spread_y = y - round(y)
            if abs(spread_y) < TOL_ALONG:
                x_wall = round(x - 0.5) + 0.5
                y_wall = round(y)
                candidates = addPoint(candidates, x_wall, y_wall)
    
        # Or a horizontal one
        elif abs(err_norm_y) < TOL_NORMAL:
            spread_x = x - round(x)
            if abs(spread_x) < TOL_ALONG:
                x_wall = round(x)
                y_wall = round(y - 0.5) + 0.5
                candidates = addPoint(candidates, x_wall, y_wall)

    new_walls = [(x,y) for (x,y,cnt) in candidates if (cnt >= TOL_NB_PTS)]
    return new_walls


def addPoint(candidates, x_wall, y_wall):
    already_detected = False
    for i in range(len(candidates)):
        if (x_wall, y_wall) == (candidates[i][0], candidates[i][1]):
            candidates[i][2] += 1 # increase number of corresponding points
            already_detected = True
            break
    if not already_detected:
        candidates.append([x_wall, y_wall, 1])
    return candidates


if __name__ == "__main__":
    pcl = [(0.4,1,0),(0.36,0.6,0), (0.5,1,0), (0.61,0.7,0),(0.36,0.7,0),(0,0.4,0),(0.3,1.36,0)]
    new_walls = extractWalls(pcl)
    print new_walls
