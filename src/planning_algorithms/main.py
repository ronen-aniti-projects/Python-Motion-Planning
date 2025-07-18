import numpy as np
from pathlib import Path
import json
from .environment_data import EnvironmentData
from .lattice import CubicLattice
from .a_star_search import astar 
from .utils import euclidean_distance
from .prm import PRM
from .trajectory import TrajectoryPlanner, Trajectory

def run_obstacle_data(obstacle_file_path, margin_of_safety):
	""""Demonstrates the obstacle data processing module
		
	Args:
		obstacle_file_path (str): The file path of the obstacle data
		margin_of_safety (float): The safety margin around obstacles
		
	Returns: 
		ObstacleData: An object encapsulating the obstacle set geometry and associated metadata
	"""
	
	print("Running the obstacle processing module...")
	enivronment_data = EnvironmentData(obstacle_file_path, margin_of_safety)
	enivronment_data.summary()
	enivronment_data.visualize()
	
	return enivronment_data

def run_lattice(environment_data, center, halfsizes, resolution, connectivity):
	"""Demonstrates the free-space construction module

	Args:
		environment_data (EnvironmentData): An object containing the processed environment data
		center (list): The center coordinates of the lattice volume
		halfsizes (list): The half-size dimensions of the lattice volume
		resolution (float): The spacing between lattice points
		connectivity (str): The type of lattice connectivity ("full" or "partial")
		
	Returns:
		CubicLattice: A graph structure encapsulating free-space 
	"""
	
	print("Building a model of free-space...")

	center_np = np.array(center)
	halfsizes_np = np.array(halfsizes)
	lattice = CubicLattice(environment_data, center_np, halfsizes_np, resolution, connectivity) 
	lattice.visualize(environment_data)
	return lattice 

def run_astar(environment_data, lattice, start_gps, goal_gps):
	"""Demonstrates the A* search module

	Args:
		environment_data (EnvironmentData): An object containing the processed environment data
		lattice (CubicLattice): A graph structure encapsulating free-space 
		start_gps (list): The starting GPS coordinates [lon, lat, alt]
		goal_gps (list): The goal GPS coordinates [lon, lat, alt]
		
	Returns:
		list: A waypoint path
	"""
	start_gps_np = np.array(start_gps)
	goal_gps_np = np.array(goal_gps)
	optimal_path = astar(lattice, start_gps_np, goal_gps_np, euclidean_distance)
	lattice.visualize(environment_data, path=optimal_path)

	return optimal_path

def run_prm(environment_data, start_gps, goal_gps, density, neighbors, visualization_bounds):
	"""Demonstrate the PRM search module

	Args:
		environment_data (EnvironmentData): An object containing the processed environment data
		start_gps (list): The starting GPS coordinates [lon, lat, alt]
		goal_gps (list): The goal GPS coordinates [lon, lat, alt]
		density (float): The target density for nodes in free-space
		neighbors (int): The branching factor for the PRM graph
		visualization_bounds (list): The bounds for the visualization [xmin, ymin, zmin, xmax, ymax, zmax]
	"""
	roadmap = PRM(environment_data, DENSITY=density, NEIGHBORS=neighbors)
	roadmap.visualize(visualization_bounds)
	prm_path = astar(roadmap, start_gps, goal_gps, euclidean_distance)
	roadmap.visualize(visualization_bounds, path=prm_path)
	
	return prm_path

def run_trajectory(waypoints, average_speed, output_directory):
	"""Demonstrate the trajectory planning module

	Args:
		waypoints (list): A list of waypoints
		average_speed (float): The average speed to enforce along the trajectory
		output_directory (str): The directory where trajectory data will be stored 
	"""
	planner = TrajectoryPlanner(waypoints)
	planner.allocate_time(average_speed)
	trajectory = planner.compute_complete_trajectory()
	print("Saving...")
	trajectory.save_trajectory_to_files('data/output/')
	print("Save complete")

	# Plot the trajectory
	trajectory.plot_3d_trajectory()
	trajectory.plot_velocity()
	trajectory.plot_acceleration()
	trajectory.plot_jerk()
	trajectory.plot_snap()
	
	# View the coefficients of the trajectory
	print(trajectory.x_dict)
	print(trajectory.y_dict)
	print(trajectory.z_dict)
	
	
def run_full_mission(config):
	"""Demonstrates an entire mission simulation
	
	Args:
		config (dict): A dictionary containing all mission parameters 
	"""
	# 1. Demonstrate the obstacle processor module
	env_config =  config['environment']
	environment_data = run_obstacle_data(
		obstacle_file_path=env_config['obstacle_file'],
		margin_of_safety=env_config['margin_of_safety'])
	
	# 2. Demonstrate the graph construction module
	lattice_config = config['lattice']
	lattice = run_lattice(environment_data=environment_data,
						  center=lattice_config['center'],
						  halfsizes=lattice_config['halfsizes'],
						  resolution=lattice_config['resolution'],
						  connectivity=lattice_config['connectivity'])
	
	# 3. Demonsrtate the A* pathfinding module
	astar_config = config['astar']
	optimal_path = run_astar(
		environment_data=environment_data,
		lattice=lattice,
		start_gps=astar_config['start_gps'],
		goal_gps=astar_config['goal_gps'])
	
	# 4. Demonstrate the PRM pathfinding module
	prm_config = config['prm']
	prm_path = run_prm(
		environment_data=environment_data,
		start_gps=prm_config['start_gps'],
		goal_gps=prm_config['goal_gps'],
		density=prm_config['density'],
		neighbors=prm_config['neighbors'],
		visualization_bounds=prm_config['visualization_bounds']
	)
 
	# 5. Demonstrate the trajectory planning module
	trajectory_config = config['trajectory']
	run_trajectory(waypoints=trajectory_config['waypoints'],
				average_speed=trajectory_config['average_speed'],
				output_directory=trajectory_config['output_directory'])
	
def main():
	
	# Define the path to the configuration file
	config_path = Path("config.json")
	
	# Load the configuration from the JSON file 
	try:
		with open(config_path, 'r') as f:
			config_data = json.load(f)
		
		# Run the main mission, passing the user config
		run_full_mission(config_data)
	
	except FileNotFoundError:
		print("Error: Configuration file not found")
	except json.JSONDecodeError:
		print("Error: Could not decode JSON")

	