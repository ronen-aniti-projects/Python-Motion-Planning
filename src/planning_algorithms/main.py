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
    """ "Demonstrates the obstacle data processing module

    Args:
            obstacle_file_path (str): The file path of the obstacle data
            margin_of_safety (float): The safety margin around obstacles

    Returns:
            ObstacleData: An object encapsulating the obstacle set geometry and associated metadata
    """
    enivronment_data = EnvironmentData(obstacle_file_path, margin_of_safety)
    
    print("\nHere is a summary of the obstacle metadata: ")
    enivronment_data.summary()
    
    print("\nGenerating 3D visualization....")
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
    center_np = np.array(center)
    halfsizes_np = np.array(halfsizes)
    lattice = CubicLattice(
        environment_data, center_np, halfsizes_np, resolution, connectivity
    )
    
    print("\nGenerating 3D visualization....")
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
    optimal_path = astar(
        lattice, start_gps_np, goal_gps_np, euclidean_distance
    )
    print("\nGenerating 3D visualization....")
    lattice.visualize(environment_data, path=optimal_path)

    return optimal_path


def run_prm(
    environment_data,
    start_gps,
    goal_gps,
    density,
    neighbors,
    visualization_bounds,
):
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
    print("\nGenerating 3D visualization....")
    roadmap.visualize(visualization_bounds)
    print("\nSearching PRM with A*...")
    prm_path = astar(roadmap, start_gps, goal_gps, euclidean_distance)
    print("\nGenerating 3D visualization....")
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
    trajectory.save_trajectory_to_files("data/output/")
    print("Save complete")

    # Plot the trajectory
    print("Generating plots of trajectory profiles")
    trajectory.plot_3d_trajectory()
    trajectory.plot_velocity()
    trajectory.plot_acceleration()
    trajectory.plot_jerk()
    trajectory.plot_snap()

def main():
    print("========== WELCOME ========== ")
    print("Welcome to a demonstration of Ronen Aniti's motion planning pipeline.")

    config_path = Path("config.json")
    config_data = None
    environment_data = None
    lattice = None
    optimal_path = None
    prm_path = None

    # ===== 1. Demonstrate loading of the configuration file ======
    while True:
        print("\n--- Step 1: Load the configuration file ---")
        print("Press [D] to load configuration")
        print("Press [E] to exit")

        user_input = input("Your choice: ").strip().lower()

        if user_input == 'd':
            try:
                with open(config_path, "r") as f:
                    config_data = json.load(f)
                print("Configuration successfully loaded.")
                break
            except FileNotFoundError:
                print("Error: Configuration file not found.")
            except json.JSONDecodeError:
                print("Error: Could not decode JSON.")
        elif user_input == 'e':
            print("Program terminating")
            return
        else:
            print("Invalid input. Please press [D] or [E].")

    # ===== 2. Demonstrate the obstacle processing module ======
    while True:
        print("\n--- Step 2: Load and visualize obstacle data ---")
        print("Press [D] to load and visualize obstacle data")
        print("Press [E] to exit")

        user_input = input("Your choice: ").strip().lower()

        if user_input == 'd':
            try:
                env_config = config_data.get("environment")
                if not env_config:
                    print("Error: Missing 'environment' config field.")
                    continue
                
                environment_data = run_obstacle_data(
                    obstacle_file_path=env_config["obstacle_file"],
                    margin_of_safety=env_config["margin_of_safety"]
                )
                break
            except Exception as e:
                print(f"Error: Failed to load obstacle data: {e}")
        elif user_input == 'e':
            print("Program terminating")
            return
        else:
            print("Invalid input. Please press [D] or [E].")
            
    # ===== 3. Demonstrate the lattice construction module ======
    while True:
        print("\n--- Step 3: Construct a map of free space ---")
        print("Press [D] to demonstrate the lattice construction module")
        print("Press [E] to exit")

        user_input = input("Your choice: ").strip().lower()

        if user_input == 'd':
            try:
                lattice_config = config_data.get("lattice")
                if not lattice_config:
                    print("Error: Missing 'lattice' config field.")
                    continue
                
                lattice = run_lattice(
                    environment_data=environment_data,
                    center=lattice_config["center"],
                    halfsizes=lattice_config["halfsizes"],
                    resolution=lattice_config["resolution"],
                    connectivity=lattice_config["connectivity"]
                )
                break
            except Exception as e:
                print(f"Error: Failed to construct lattice: {e}")
        elif user_input == 'e':
            print("Program terminating")
            return
        else:
            print("Invalid input. Please press [D] or [E].")
               
    # ===== 4. Demonstrate the A* search module ======
    while True:
        print("\n--- Step 4: Search for a navigable route with A* ---")
        print("Press [D] demonstrate the A* search module")
        print("Press [E] to exit")

        user_input = input("Your choice: ").strip().lower()

        if user_input == 'd':
            try:

                astar_config = config_data.get("astar")
                if not astar_config:
                    print("Error: Missing 'astar' config section.")
                    continue

                optimal_path = run_astar(
                    environment_data=environment_data,
                    lattice=lattice,
                    start_gps=astar_config["start_gps"],
                    goal_gps=astar_config["goal_gps"]
                )
                print("A* pathfinding completed.")
                break

            except Exception as e:
                print(f"A* search failed: {e}")

        elif user_input == 'e':
            print("Program terminating")
            return

        else:
            print("Invalid input. Please press [D] or [E].")
    
    # ===== 5. Demonstrate the PRM module ======
    while True:
        print("\n--- Step 5: Plan a path with PRM ---")
        print("Press [D] to execute PRM and visualize the roadmap")
        print("Press [E] to exit")

        user_input = input("Your choice: ").strip().lower()

        if user_input == 'd':
            try:
                prm_config = config_data.get("prm")
                if not prm_config:
                    print("Error: Missing 'prm' config section.")
                    continue

                prm_path = run_prm(
                    environment_data=environment_data,
                    start_gps=prm_config["start_gps"],
                    goal_gps=prm_config["goal_gps"],
                    density=prm_config["density"],
                    neighbors=prm_config["neighbors"],
                    visualization_bounds=prm_config["visualization_bounds"]
                )
                print("PRM search completed.")
                break

            except Exception as e:
                print(f"PRM search failed: {e}")

        elif user_input == 'e':
            print("Program terminating")
            return

        else:
            print("Invalid input. Please press [D] or [E].")
            
    # ===== 6. Demonstrate the trajectory module ======
    while True:
        print("\n--- Step 6: Generate a trajectory between waypoints ---")
        print("Press [D] to generate and visualize trajectory")
        print("Press [E] to exit")

        user_input = input("Your choice: ").strip().lower()

        if user_input == 'd':
            try:
                trajectory_config = config_data.get("trajectory")
                if not trajectory_config:
                    print("Error: Missing 'trajectory' config section.")
                    continue

                run_trajectory(
                    waypoints=trajectory_config["waypoints"],
                    average_speed=trajectory_config["average_speed"],
                    output_directory=trajectory_config["output_directory"]
                )
                print("Trajectory generated and saved.")
                break

            except Exception as e:
                print(f"Trajectory planning failed: {e}")

        elif user_input == 'e':
            print("Program terminating")
            return

        else:
            print("Invalid input. Please press [D] or [E].")
