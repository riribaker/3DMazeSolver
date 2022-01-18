# geometry.py
# ---------------
# Licensing Information:  You are free to use or extend this projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to the University of Illinois at Urbana-Champaign
#
# Created by James Gao (jamesjg2@illinois.edu) on 9/03/2021
# Inspired by work done by Jongdeog Lee (jlee700@illinois.edu)

"""
This file contains geometry functions necessary for solving problems in MP2
"""

import math
import numpy as np
from alien import Alien


def does_alien_touch_wall(alien, walls,granularity):
    """Determine whether the alien touches a wall

        Args:
            alien (Alien): Instance of Alien class that will be navigating our map
            walls (list): List of endpoints of line segments that comprise the walls in the maze in the format [(startx, starty, endx, endx), ...]
            granularity (int): The granularity of the map

        Return:
            True if touched, False if not
    """
    # Debugging: print(alien.get_head_and_tail())
    
    aR = alien.get_width()
    buf = aR + (granularity / np.sqrt(2))
    positions = []
    if(alien.is_circle()):
        positions.append(np.array(alien.get_centroid()))
    else:
        positions = np.array(alien.get_head_and_tail())
    
    for w in walls:
        w1 = np.array([w[0],w[1]])
        w2 = np.array([w[2],w[3]])
        for p in positions:
            distToWall = dist_pt_to_line(p,[w1,w2])
            if(distToWall < buf or np.isclose(distToWall,buf)):
                return True
        if(not alien.is_circle()):
            distToCorner = dist_pt_to_line(w1,positions)
            distToCorner2 = dist_pt_to_line(w2,positions)
            if(distToCorner < buf or np.isclose(distToCorner,buf) or
               distToCorner2 < buf or np.isclose(distToCorner2,buf)):
                return True
        # detect line collision for oblongs 
        # horizontal = 0, vertical = 2
        if((alien.get_shape() == 'Horizontal' and positions[0][1] <= max(w[1],w[3]) and positions[0][1]>= min(w[1],w[3]))
            or (alien.get_shape() == 'Vertical' and positions[0][0] <= max(w[0],w[2]) and positions[0][0]>= min(w[0],w[2]))):
            if(does_intersect([w1,w2],positions)):
                return True
            
    return False

def does_alien_touch_goal(alien, goals):
    """Determine whether the alien touches a goal
        
        Args:
            alien (Alien): Instance of Alien class that will be navigating our map
            goals (list): x, y coordinate and radius of goals in the format [(x, y, r), ...]. There can be multiple goals
        
        Return:
            True if a goal is touched, False if not.
    """
    aR = alien.get_width()
    positions = []
    if(alien.is_circle()):
        positions.append(np.array(alien.get_centroid()))
    else:     
        positions = np.array(alien.get_head_and_tail())
    
    # check that distance from any position (centroids) is less than or equal to radius of alien + radius of goal
    for g in goals:
        gR = g[2]
        goal = np.array([g[0],g[1]])
        for p in positions:
            dist = np.linalg.norm(p-goal)
            if(dist < (aR + gR) or np.isclose(dist,(aR + gR))):
                return True                    
        # for oblong form, need to check dist between line and goal centroid
        # only check for vertical oblong if goal Y coord between Y coordinates of head&tail
        # only check for horizontal oblong if goal X coord between X coordinates of head&tail
        if((alien.get_shape_idx() == 2 and goal[1]>positions[0][1] and goal[1]<positions[1][1])
            or (alien.get_shape_idx() == 0 and goal[0]<positions[0][0] and goal[0]>positions[1][0])):
                
                eq1 = np.cross(positions[1]-positions[0],positions[0]- goal) / np.linalg.norm(positions[1]-positions[0])
                lineDist = np.linalg.norm(eq1)
                if(lineDist < (aR + gR) or np.isclose(lineDist,(aR + gR))):
                    return True
    return False

def is_alien_within_window(alien, window,granularity):
    """Determine whether the alien stays within the window
        
        Args:
            alien (Alien): Alien instance
            window (tuple): (width, height) of the window
            granularity (int): The granularity of the map
    """
    winX = window[0]
    winY = window[1]
    radius = alien.get_width()
    buf = radius + (granularity / np.sqrt(2))

    if(alien.is_circle()):
        pos = alien.get_centroid()
        x = pos[0]
        y = pos[1]

        if    ((x-buf) < 0 or np.isclose((x-buf),0) 
            or (x+buf) > winX or np.isclose((x+buf),winX)
            or (y-buf) < 0 or np.isclose((y-buf),0)
            or (y+buf) > winY or np.isclose((y+buf),winY)):
            return False
    else:
        pos = alien.get_head_and_tail()
        headX = pos[0][0]
        headY = pos[0][1]
        tailX = pos[1][0]
        tailY = pos[1][1]

        if    ((headX-buf) < 0 or np.isclose((headX-buf),0) 
            or (headX+buf) > winX or np.isclose((headX+buf),winX)
            or (headY-buf) < 0 or np.isclose((headY-buf),0)
            or (headY+buf) > winY or np.isclose((headY+buf),winY)):
            return False
        if    ((tailX-buf) < 0 or np.isclose((tailX-buf),0) 
            or (tailX+buf) > winX or np.isclose((tailX+buf),winX)
            or (tailY-buf) < 0 or np.isclose((tailY-buf),0)
            or (tailY+buf) > winY or np.isclose((tailY+buf),winY)):
            return False

    return True

# returns distance from point to line segment
def dist_pt_to_line(pt,line):
    #pt1: np.array(x,y)
    #line: np.array([x1,y1],[x2,y2])
    
    a = line[0] - pt
    b = line[0] - line[1]
    c = line[1] - pt
    lineNorm = np.linalg.norm(b)
    dotdist = np.dot(a,b)/np.linalg.norm(b)
    # not within range of segment, closer to second point in line
    if(dotdist > lineNorm):
        return np.linalg.norm(c)
    # not within range of segment, closest to first point in line
    if(dotdist < 0):
        return np.linalg.norm(a)
    # otherwise within range, find distance to closest point on segment
    return np.abs(np.cross(a,b)/lineNorm)

def does_intersect(wall,oblong):
    # returns true if wall intersects with oblong
    a = wall[0] - oblong[0]
    a2 = wall[0] - oblong[1]
    b = wall[0] - wall[1]
    wallNorm = np.linalg.norm(b)
    sin1 = np.cross(a,b) / (np.linalg.norm(a) * wallNorm)
    sin2 = np.cross(a2,b)/ (np.linalg.norm(a2)* wallNorm)
    return(np.sign(sin1)!=np.sign(sin2))
    
if __name__ == '__main__':
    #Walls, goals, and aliens taken from Test1 map
    walls =   [(0,100,100,100),  
                (0,140,100,140),
                (100,100,140,110),
                (100,140,140,130),
                (140,110,175,70),
                (140,130,200,130),
                (200,130,200,10),
                (200,10,140,10),
                (175,70,140,70),
                (140,70,130,55),
                (140,10,130,25),
                (130,55,90,55),
                (130,25,90,25),
                (90,55,90,25)]
    goals = [(110, 40, 10)]
    window = (220, 200)

    def test_helper(alien : Alien, position, truths):
        alien.set_alien_pos(position)
        config = alien.get_config()

        touch_wall_result = does_alien_touch_wall(alien, walls, 0) 
        touch_goal_result = does_alien_touch_goal(alien, goals)
        in_window_result = is_alien_within_window(alien, window, 0)

        assert touch_wall_result == truths[0], f'does_alien_touch_wall(alien, walls) with alien config {config} returns {touch_wall_result}, expected: {truths[0]}'
        assert touch_goal_result == truths[1], f'does_alien_touch_goal(alien, goals) with alien config {config} returns {touch_goal_result}, expected: {truths[1]}'
        assert in_window_result == truths[2], f'is_alien_within_window(alien, window) with alien config {config} returns {in_window_result}, expected: {truths[2]}'

    #Initialize Aliens and perform simple sanity check. 
    alien_ball = Alien((30,120), [40, 0, 40], [11, 25, 11], ('Horizontal','Ball','Vertical'), 'Ball', window)
    test_helper(alien_ball, alien_ball.get_centroid(), (False, False, True))

    alien_horz = Alien((30,120), [40, 0, 40], [11, 25, 11], ('Horizontal','Ball','Vertical'), 'Horizontal', window)	
    test_helper(alien_horz, alien_horz.get_centroid(), (False, False, True))

    alien_vert = Alien((30,120), [40, 0, 40], [11, 25, 11], ('Horizontal','Ball','Vertical'), 'Vertical', window)	
    test_helper(alien_vert, alien_vert.get_centroid(), (True, False, True))

    edge_horz_alien = Alien((50, 100), [100, 0, 100], [11, 25, 11], ('Horizontal','Ball','Vertical'), 'Horizontal', window)
    edge_vert_alien = Alien((200, 70), [120, 0, 120], [11, 25, 11], ('Horizontal','Ball','Vertical'), 'Vertical', window)

    alien_positions = [
                        #Sanity Check
                        (0, 100),

                        #Testing window boundary checks
                        (25.6, 25.6),
                        (25.5, 25.5),
                        (194.4, 174.4),
                        (194.5, 174.5),

                        #Testing wall collisions
                        (30, 112),
                        (30, 113),
                        (30, 105.5),
                        (30, 105.6), # Very close edge case
                        (30, 135),
                        (140, 120),
                        (187.5, 70), # Another very close corner case, right on corner
                        
                        #Testing goal collisions
                        (110, 40),
                        (145.5, 40), # Horizontal tangent to goal
                        (110, 62.5), # ball tangent to goal
                        
                        #Test parallel line oblong line segment and wall
                        (50, 100),
                        (200, 100),
                        (205.5, 100) #Out of bounds
                    ]

    #Truths are a list of tuples that we will compare to function calls in the form (does_alien_touch_wall, does_alien_touch_goal, is_alien_within_window)
    alien_ball_truths = [
                            (True, False, False),
                            (False, False, True),
                            (False, False, True),
                            (False, False, True),
                            (False, False, True),
                            (True, False, True),
                            (False, False, True),
                            (True, False, True),
                            (True, False, True),
                            (True, False, True),
                            (True, False, True),
                            (True, False, True),
                            (False, True, True),
                            (False, False, True),
                            (True, True, True),
                            (True, False, True),
                            (True, False, True),
                            (True, False, True)
                        ]
    alien_horz_truths = [
                            (True, False, False),
                            (False, False, True),
                            (False, False, False),
                            (False, False, True),
                            (False, False, False),
                            (False, False, True),
                            (False, False, True),
                            (True, False, True),
                            (False, False, True),
                            (True, False, True),
                            (False, False, True),
                            (True, False, True),
                            (True, True, True),
                            (False, True, True),
                            (True, False, True),
                            (True, False, True),
                            (True, False, False),
                            (True, False, False)
                        ]
    alien_vert_truths = [
                            (True, False, False),
                            (False, False, True),
                            (False, False, False),
                            (False, False, True),
                            (False, False, False),
                            (True, False, True),
                            (True, False, True),
                            (True, False, True),
                            (True, False, True),
                            (True, False, True),
                            (True, False, True),
                            (False, False, True),
                            (True, True, True),
                            (False, False, True),
                            (True, True, True),
                            (True, False, True),
                            (True, False, True),
                            (True, False, True)
                        ]

    for i in range(len(alien_positions)):
        test_helper(alien_ball, alien_positions[i], alien_ball_truths[i])
        test_helper(alien_horz, alien_positions[i], alien_horz_truths[i])
        test_helper(alien_vert, alien_positions[i], alien_vert_truths[i])

    #Edge case coincide line endpoints
    test_helper(edge_horz_alien, edge_horz_alien.get_centroid(), (True, False, False))
    test_helper(edge_horz_alien, (110,55), (True, True, True))
    test_helper(edge_vert_alien, edge_vert_alien.get_centroid(), (True, False, True))


    print("Geometry tests passed\n")