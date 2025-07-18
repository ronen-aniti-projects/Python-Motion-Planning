# main.py
import numpy as np
from planning_module.environment_data import EnvironmentData
from planning_module.lattice import CubicLattice
from planning_module.a_star_search import astar 
from planning_module.utils import euclidean_distance

def run_obstacle_data(obstacle_file_path):
    print("Building, summarizing, and visualizing the 3D environment...")
    
    # Constructs an EnvironmentData object, parsing the obstacle data from 'colliders.csv' and setting the margin of safety to 5.0
    enivronment_data = EnvironmentData(obstacle_file_path, 5.0)
    
    # Prints a summary of the environment data key features
    enivronment_data.summary()
    
    # Visualizes the environment data in 3D
    print("Plotting the 3D environment")
    enivronment_data.visualize()
    
    return enivronment_data

def run_lattice(environment_data):
    print("Building a model of free-space...")

    # Create a CubicLattice object, provided the specified center and bounding box dimensions of the volume to model
    center = np.array([0, 0, 0])
    halfsizes = np.array([50, 50, 50])
    lattice = CubicLattice(environment_data, center, halfsizes, resolution=15.0, connectivity="partial") 
    
    # Visualize the lattice
    # Options: connectivity="full" or "partial":
    # connectivity="full" creates a 26-connected lattice, where each point is connected to up to 26 neighbors of its neighbors
    # connectivity="partial" creates a 6-connected lattice, where each point is connected to up to 6 neighbors
    lattice.visualize(environment_data)
    
    return lattice 

def run_astar(lattice):

    # Construct a free space lattice with a safety margin of 5.0 m
    env_data = EnvironmentData("data/input/colliders.csv", 5.0)
    
    # Define the start and goal GPS positions
    start_gps = np.array([-122.397450, 37.792480, 0.0])
    goal_gps = np.array([-122.397230, 37.792895, 50.0])
    
    # Perform A* search to find a path from the start to the goal position
    optimal_path = astar(lattice, start_gps, goal_gps, euclidean_distance)
    
    # Visualize the path in the free space lattice
    lattice.visualize(env_data, path=optimal_path)

    return optimal_path

def run_full_mission():
    """
    This function represents a complete, integrated mission.
    """
    
    obstacle_file_path = "data/input/colliders.csv"
    
    # Run the 3D environment build module
    environment_data = run_obstacle_data(obstacle_file_path)
    
    # Run the lattice construction module 
    lattice = run_lattice(environment_data)
    
    # Run the A* search module 
    optimal_path = run_astar(lattice)
    
    # Run the PRM module

	# Start and goal GPS coordinates
	start_gps = [-122.39745, 37.79248, 0]
	goal_gps = [-122.39645,  37.79278, 200]
	
	# Generate a PRM
	roadmap = PRM(env_data, DENSITY=1e-5, NEIGHBORS=5)
	
	# Visualize the PRM
	roadmap.visualize([-100, 100, -100, 100, 0, 200])
	
	# Perform A* search on the PRM
	path = astar(roadmap, start_gps, goal_gps, euclidean_distance)
	
	# Visualize the results
	roadmap.visualize([-100, 100, -100, 100, 0, 200], path=path)
    
    
if __name__ == "__main__":
    run_full_mission()
    
    