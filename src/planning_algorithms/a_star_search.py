"""
This module is an implementation of the A* search algorithm.
"""

import heapq

import numpy as np

from .environment_data import EnvironmentData
from .lattice import CubicLattice
from .utils import (
    euclidean_distance,
    find_nearest,
    global_to_local,
    valid_neighbors,
)


def astar(free_space_lattice_object, start_gps, goal_gps, h):
    """
    Perform A* search in the free space lattice to find a path from the start position to the goal position.

    Args:
            free_space_lattice_object (Lattice object): an object of the CubicLattice class
           start_gps (numpy.ndarray): a numpy array of shape (3,) containing the start GPS position
           goal_gps (numpy.ndarray): a numpy array of shape (3,) containing the goal GPS position
           h (function): the heuristic function to use for A* search

    Returns:
            list: a list of indices representing the path from the start position to the goal position

    Note:
            The A* search algorithm uses a priority queue to explore the lattice space. The algorithm continues to explore the lattice space until the goal position is found or all points have been visited.
           The benefits of this approach for pathfinding are that it is guaranteed to find the shortest path and is complete if the heuristic is admissible.
           On the other hand, the A* search algorithm can be slow and memory-intensive for large lattices due to the way it explores the lattice space.
    """
    start_pos = global_to_local(start_gps, free_space_lattice_object.gps_home)
    start_index = find_nearest(free_space_lattice_object, start_pos)
    goal_pos = global_to_local(goal_gps, free_space_lattice_object.gps_home)
    goal_index = find_nearest(free_space_lattice_object, goal_pos)

    goal_found = False
    branch = {}
    branch[start_index] = [None, 0.0]
    visited = set()

    priority_queue = []
    heapq.heappush(priority_queue, (0.0, start_index))

    while priority_queue:
        _, current_index = heapq.heappop(priority_queue)
        if current_index in visited:
            continue
        visited.add(current_index)
        if current_index == goal_index:
            print("Goal found")
            goal_found = True
            break
        for neighbor_index, distance in valid_neighbors(
            free_space_lattice_object, current_index
        ):
            if neighbor_index in visited:
                continue
            g_score = branch[current_index][1]
            g_score_tentative = g_score + distance
            h_score_tentative = h(
                free_space_lattice_object, neighbor_index, goal_index
            )
            f_score_tentative = g_score_tentative + h_score_tentative
            if neighbor_index not in branch.keys() or (
                (neighbor_index in branch.keys())
                and g_score_tentative <= branch[neighbor_index][1]
            ):
                heapq.heappush(
                    priority_queue, (f_score_tentative, neighbor_index)
                )
                branch[neighbor_index] = [current_index, g_score_tentative]

    if goal_found:
        cost = branch[goal_index][1]
        path = []
        while current_index != start_index:
            path.append(current_index)
            current_index = branch[current_index][0]
        path.append(start_index)
        path = path[::-1]

    return path

