import numpy as np
import utm 

"""
This file contains functions for checking collisions between a point and an environment.
The reason for the different collision check functions is to compare the performance of different methods.
"""

def collision_check_basic(environment_data_object, point):
    """
    Check if a point is in collision with the environment.
    
    Args:
        environment_data_object (EnvironmentData): an object of the EnvironmentData class
        point (numpy.ndarray): a numpy array of shape (3,) containing the point to check
        
    Returns:
        bool: a boolean indicating whether the point is in collision with the environment
    
    Note:
        This function uses a `for` loop to iterate over the obstacles in the environment and check if the point is in collision with any of them.
        The benefits of this function are that it is simple to implement and understand.
        The drawbacks are that it is not vectorized and may be slow for large environments with many obstacles.
    """
    for idx, obstacle_center in enumerate(environment_data_object.centers):
        if abs(point[0] - obstacle_center[0]) <= environment_data_object.halfsizes[idx, 0]:
            if abs(point[1] - obstacle_center[1]) <= environment_data_object.halfsizes[idx, 1]:
                if abs(point[1] - obstacle_center[2]) <= environment_data_object.halfsizes[idx, 2]:
                    return True
    return False

def collision_check_vectorized(environment_data_object, point):
    """
    Check if a point is in collision with the environment.    
    
    Args:
        environment_data_object (EnvironmentData): an object of the EnvironmentData class
        point (numpy.ndarray): a numpy array of shape (3,) containing the point to check
        
    Returns:
        bool: a boolean indicating whether the point is in collision with the environment
        
    Note:
        This function uses vectorized operations to check if the point is in collision with any of the obstacles in the environment.
        The benefits of this function are that it is more efficient than the basic collision check function and is easy to understand.
        The drawbacks are that it may be slower than the KDTree collision check function for some environments.
    """
    
    broadcasted_point = np.tile(point, (len(environment_data_object.centers),1))
    deltas = np.abs(broadcasted_point - environment_data_object.centers)
    collision_conditions = (deltas <= environment_data_object.halfsizes)
    return np.any(np.all(collision_conditions, axis=1))

def collision_check_spatial(environment_data_object, point):
    """
    Check if a point is in collision with the environment.
    
    Args:
        environment_data_object (EnvironmentData): an object of the EnvironmentData class
        point (numpy.ndarray): a numpy array of shape (3,) containing the point to check
        
    Returns: 
        bool: a boolean indicating whether the point is in collision with the environment
    
    Note: 
        This function uses a KDTree to query the nearest obstacles to the point and check if the point is in collision with any of them.
        The benefits of this function are that it is more efficient than the basic collision check function and is easy to understand.
        The drawbacks are that it may be slower than the vectorized collision check function for some environments.
        Also, the KDTree query may return false positives if the point is close to an obstacle but not in collision with it.
        Furthermore, the number of nearest obstacles to query is hardcoded to 4, which may not be optimal for all environments.
    """
    _, indices = environment_data_object.ground_centers.query([point[:2]], k=4)
    for i in indices[0]:
        if environment_data_object.heights[i] >= point[2]:
            if abs(environment_data_object.centers[i][0] - point[0]) <= environment_data_object.halfsizes[i][0]:
                if abs(environment_data_object.centers[i][1] - point[1]) <= environment_data_object.halfsizes[i][1]:
                    return True
    return False

def inside_environment(environment_data_object, point):
    """
    Check if a point is inside the environment.
    
    Args:
        environment_data_object (EnvironmentData): an object of the EnvironmentData class
        point (numpy.ndarray): a numpy array of shape (3,) containing the point to check
        
    Returns:
        bool: a boolean indicating whether the point is inside the environment
    """
    if point[0] < environment_data_object.xbounds[0] or point[0] > environment_data_object.xbounds[1]:
        return False
    if point[1] < environment_data_object.ybounds[0] or point[1] > environment_data_object.ybounds[1]:
        return False
    if point[2] < environment_data_object.zbounds[0] or point[2] > environment_data_object.zbounds[1]:
        return False
    return True

def collision_check_two_points(environment_data_object, point1, point2, SPACING=1.0):
    """
    Check if the line segment between two points is in collision with the environment.
    
    Args:
        environment_data_object (EnvironmentData): an object of the EnvironmentData class
        point1 (numpy.ndarray): a numpy array of shape (3,) containing the start point of the line segment
        point2 (numpy.ndarray): a numpy array of shape (3,) containing the end point of the line segment
        SPACING (float): the spacing between test points on the line segment
        
    Returns:
        bool: a boolean indicating whether the line segment is in collision with the environment
    
    Note:
        The way this function works is by dividing the line segment into a number of test points based on the spacing parameter.
        It then checks if any of the test points are in collision with the environment.
        The benefits of this function are that it is simple to implement and understand.
        The drawbacks are that it may be slow for large line segments or small spacing values.
    """
    delta = point2 - point1
    distance = np.linalg.norm(delta)
    direction = delta / distance
    number_of_points = int(distance / SPACING)
    test_points = np.array([point1 + i * SPACING * direction for i in range(number_of_points + 1)])
    segment_evaluation = np.array([collision_check_vectorized(environment_data_object, test_point) for test_point in test_points])
    return np.any(segment_evaluation)


def global_to_local(global_position, global_home):
    """
    Inputs: global_position: numpy array [longitude, latitude, altitude]; global_home: numpy array: [longitude, latitude, 
        altitude]
    Returns: local_position numpy array [northing, easting, altitude] of global_position
        relative to global_home [longitude, latitude, altitude]; Essentially, returns a
        numpy array describing the NED delta from global_home.
    """
    # Get easting and northing of global home first
    lonh, lath, alth = global_home[0], global_home[1], global_home[2]
    (easting_h, northing_h, zone_number_h, zone_letter_h) = utm.from_latlon(lath, lonh)
    
    # Get easting and northing from global position next
    lon, lat, alt = global_position[0], global_position[1], global_position[2]
    (easting, northing, zone_number, zone_letter) = utm.from_latlon(lat, lon)
    
    # After that, create a local_position numpy array from its NED coordinates
    local_position = np.array([northing - northing_h, easting - easting_h, (alt - alth)])
    
    # Finally, return them.
    return local_position


def local_to_global(local_position, global_home):
    """
    Inputs: local_position: numpy array [northing, easting, down]; global_home: numpy array [longitude, latitude, altitude]
    Returns: global_position: numpy array [longitude, latitude, altitude]. Essentially, returns a numpy array holding a new
        geodetic location given a home geodetic location and an NED delta. 
    """
    # First, get global_home's easting, northing, zone_number, and grid_letter.
    lon_h, lat_h, alt_h = global_home[0], global_home[1], global_home[2]
    (easting_h, northing_h, zone_number_h, zone_letter_h) = utm.from_latlon(lat_h, lon_h)
    
    # After that, compute the new NED location by adding local_position to home's NED coordinates.
    northing, easting, alt = local_position[0], local_position[1], local_position[2] 
    
    # Convert the new NED location to the geodetic frame next. 
    (latitude, longitude) = utm.to_latlon(easting + easting_h, northing + northing_h, zone_number_h, zone_letter_h)
    altitude = -(alt - alt_h)
    global_position = np.array([longitude, latitude, altitude])
    
    # Finally, return the new geodetic location.
    return global_position



def euclidean_distance(lattice_object, lattice_index_1, lattice_index_2):
	"""
	Calculate the Euclidean distance between two points in the free space lattice.
	
	Args:
		lattice_object (Lattice object): an object of the CubicLattice class
        lattice_index_1 (int): the index of the first point in the free space lattice
        lattice_index_2 (int): the index of the second point in the free space lattice
		
	Returns:
		float: the Euclidean distance between the two points
	"""
	return np.linalg.norm(lattice_object.free_space_points[lattice_index_1] - lattice_object.free_space_points[lattice_index_2])


def find_nearest(free_space_lattice_object, query_pos):
	"""
	Find the nearest point in the free space lattice to the query position.
	
	Args:
		free_space_lattice_object (Lattice object): an object of the CubicLattice class
		query_pos (numpy.ndarray): a numpy array of shape (3,) containing the query position
		
	Returns: 
		int: the index of the nearest point in the free space lattice
	"""
	_, indices = free_space_lattice_object.free_space_points_kd_tree.query([query_pos], k=1)
	return indices[0]

def shortcut(environment_data_object, input_path, points):
	"""	
	Removes unnecessary waypoints from the path while keeping the path collision-free.
	
	Args:
		environment_data_object (EnvironmentData): an object of the EnvironmentData class
        input_path (list): a list of integers representing the indices of the waypoints in the path
        points (numpy.ndarray): a numpy array of shape (n,3) containing the coordinates of the waypoints
		
	Returns:
		list: a list of integers representing the indices of the waypoints in the shorter path
		
	
	Note:
		The way the algorithm works is by checking if the line between two points is in collision with the environment. If it is not, the algorithm removes the intermediate points between the two points.
		The algorithm starts by checking the line between the first and last points in the path. If the line is in collision with the environment, the algorithm moves the last point one step closer to the first point and checks again.
	
	
	"""
 
    # TODO: Refactor with readable variables names. 
	E = len(input_path) - 1
	S = 0

	e = E
	s = S

	P = [E]

	while s < e:
		shortcut_found = False
		while s < e:
			if collision_check_two_points(environment_data_object, points[input_path[s]],points[input_path[e]]):
				s += 1
			else:
				P.append(s)
				shortcut_found = True
				break
		if shortcut_found:
			e = P[-1]
			s = 0
		else:
			e -= 1
			P.append(e)
			s = 0


	P = P[::-1]
	shorter_path = [input_path[p] for p in P]
	

	return shorter_path

def valid_neighbors(free_space_lattice_object, current_index):
	"""
	Return the valid neighbors of a point in the free space lattice.
	
	Args:
		free_space_lattice_object (Lattice object): an object of the CubicLattice class
        current_index (int): the index of the current point in the free space lattice
		
	Returns:
		list: a list of tuples containing the neighbor index and the distance to the neighbor
	"""
	
	neighbors = free_space_lattice_object.graph[current_index]
	neighbor_pairs = [(neighbor_index, distance) for neighbor_index, distance in neighbors.items()]
	return neighbor_pairs 