# search.py
# ---------------
# Licensing Information:  You are free to use or extend this projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to the University of Illinois at Urbana-Champaign
#
# Created by Jongdeog Lee (jlee700@illinois.edu) on 09/12/2018

"""
This file contains search functions.
"""
# Search should return the path and the number of states explored.
# The path should be a list of tuples in the form (alpha, beta, gamma) that correspond
# to the positions of the path taken by your search algorithm.
# Number of states explored should be a number.
# maze is a Maze object based on the maze from the file specified by input filename
# searchMethod is the search method specified by --method flag (bfs)
# You may need to slight change your previous search functions in MP1 since this is 3-d maze

from collections import deque
import heapq
from heapq import heappop, heappush

def search(maze, searchMethod):
    return {
        "bfs": bfs,
    }.get(searchMethod, [])(maze)

def bfs(maze, ispart1=False):
    # Write your code here
    """
    This function returns optimal path in a list, which contains start and objective.
    If no path found, return None. 

    Args:
        maze: Maze instance from maze.py
        ispart1: pass this variable when you use functions such as getNeighbors and isObjective. DO NOT MODIFY THIS
    """
    path = []
    start = maze.getStart()
    
    # check if start = goal
    if maze.isObjective(start[0],start[1],start[2],ispart1):
        path.append(start)
        return path

    # visited states
    visited = set()
    visited.add(start)

    # queue 
    frontier = deque([start])
    
    # map to previous state
    prev = {}

    # traversal
    while frontier:
        ### val = input("continue? ")
        s = frontier.popleft()
        ### print(s)
        neighbors = maze.getNeighbors(s[0],s[1],s[2],ispart1)

        for pos in neighbors:                  # Check if neighbor is waypoint, if not add to queue
            if maze.isObjective(pos[0],pos[1],pos[2],ispart1):
                prev[pos] = s
                path = [pos]
                while path[-1] != start:
                    path.append(prev[path[-1]])
                path.reverse()
                #print(path)
                return path
                #print("figure out how to map path here")
            if pos not in visited and pos not in frontier:
                prev[pos] = s
                frontier.append(pos)
                visited.add(pos)
    return None