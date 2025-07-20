import numpy as np
import pytest

from planning_algorithms.a_star_search import astar
from planning_algorithms.utils import euclidean_distance

def test_astar_finds_path(env_data, lattice):
    """
    Tests that the A* algorithm can find a path between two points in a valid environment.
    """
    start_gps_np = np.array([-122.397450, 37.792480, 0.0])
    goal_gps_np = np.array([-122.397230, 37.792895, 50.0])
    optimal_path = astar(
        lattice, start_gps_np, goal_gps_np, euclidean_distance
    )
    assert len(optimal_path) > 1
    