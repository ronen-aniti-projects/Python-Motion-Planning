import numpy as np
from scipy.spatial import KDTree
from environment_data import EnvironmentData
from lattice import CubicLattice
from utils import global_to_local, collision_check_two_points, collision_check_vectorized, inside_environment
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class RRT:
	"""
	A class to represent a Rapidly-exploring Random Tree (RRT) for path planning in 3D space.
	RRT is a tree-based algorithm that incrementally builds a graph of states in the configuration space.
	This particular implementation incorporates hypothetical kinematic constraints on the motion of the quadcopter.
	The steering model, which I devised, is characterized by a maximum steering angle rate and a fixed speed.
	The drone's state is thus not only a position but also an orientation in 3D space, with an initial orientation that defaults
	to point directly at the goal state. 
	There is a steering axis about which the quadcopter can rotate, and the steering angle is limited by the maximum turn rate.
	Other features of this implementation include a goal bias, a maximum number of iterations, and a goal tolerance.	
	"""
	def __init__(self, environment_data_object, start_gps, goal_gps, GOAL_BIAS=0.5, MAX_STEER_ANGLE_RATE=np.pi/24, TIME_STEP=0.1, 
	    TIME_INTERVAL=5.0, SPEED=2.0, MAX_ITERATIONS=10000, GOAL_TOLERANCE=1.0):
		
		"""
		Constructor for the RRT class.
		
		Args:
			environment_data_object (EnvironmentData): An object of the EnvironmentData class.
			start_gps (numpy.ndarray): A numpy array of shape (3,) containing the starting GPS coordinates.
			goal_gps (numpy.ndarray): A numpy array of shape (3,) containing the goal GPS coordinates.
			GOAL_BIAS (float): The probability of sampling the goal state.
			MAX_STEER_ANGLE_RATE (float): The maximum steering angle rate in radians per iteration.
			TIME_STEP (float): The time step in seconds.
			TIME_INTERVAL (float): The time interval in seconds.
			SPEED (float): The speed of the vehicle in m/s.
			MAX_ITERATIONS (int): The maximum number of iterations.
			GOAL_TOLERANCE (float): The goal tolerance in meters.
			STEERING (bool): A boolean indicating whether to use steering or not.
		
		"""
		
		self._environment = environment_data_object
		self._start_gps = start_gps
		self._goal_gps = goal_gps
		self._GOAL_BIAS = GOAL_BIAS
		self._MAX_STEER_ANGLE = MAX_STEER_ANGLE_RATE * TIME_STEP # Radians per iteration
		self._TIME_STEP = TIME_STEP
		self._SPEED = SPEED
		self._MAX_ITERATIONS = MAX_ITERATIONS
		self._GOAL_TOLERANCE = GOAL_TOLERANCE
		self._TIME_INTERVAL = TIME_INTERVAL
		self._INTEGRATION_STEPS = int(self._TIME_INTERVAL / self._TIME_STEP) # Iterations of each branch expansion process
		self._STEERING = True
		self._goal_is_found = False

		# Convert GPS positions to NED coordinates
		self._start_pos = global_to_local(start_gps, environment_data_object.gps_home)
		self._goal_pos = global_to_local(goal_gps, environment_data_object.gps_home)

		# Check is positions are in free space
		if collision_check_vectorized(self._environment, self._start_pos):
			raise ValueError("Start position is not in free space")
		if collision_check_vectorized(self._environment, self._goal_pos):
			raise ValueError("Goal position is not in free space")
		if not inside_environment(self._environment, self._start_pos):
			raise ValueError("Start position is outside of the bounds of the environment")
		if not inside_environment(self._environment, self._goal_pos):
			raise ValueError("Goal position is outside of the bounds of the environment")

		# Define a starting orientation
		# Point towards the goal state, initially
		delta = self._goal_pos - self._start_pos
		start_orientation = delta / np.linalg.norm(delta)

		# Define a start state (position and orientation)
		start_state = np.concatenate((self._start_pos, start_orientation))

		# Make the start state the root of a tree
		self._states = np.array([start_state])
		self._edges = {0: None}
		


	def run(self):
		"""
		Run the RRT algorithm to find a path from the start state to the goal state.
		The algorithm will try to find a path within the maximum number of iterations.
		If the goal state is not reached, the algorithm will try again.
		
		Returns:
			list: A list of states representing the path from the start state to the goal state.
			
		"""

		for _ in range(self._MAX_ITERATIONS):
			
			# Sample a point in the environment with a bias towards the goal state
			p_sample = self.sample_with_bias()
			
			# Find the nearest state in the tree to the sample point
			x_near, index_near = self.find_nearest_state(p_sample)
			
			# Integrate the state forward from x_near towards p_sample
			x_new = self.integrate_forward(x_near, p_sample)
			
			# Check if the goal state has been reached
			if self._goal_is_found:
				
				# Add the new state to the tree
				self._states = np.vstack([self._states, x_new]) 
				index_of_new = len(self._states) - 1
				self._edges[index_of_new] = index_near 
				break
			
			# Add the new state to the tree
			self._states = np.vstack([self._states, x_new])
			index_of_new = len(self._states) - 1 
			self._edges[index_of_new] = index_near

		# If the goal state has been reached, find the path
		if self._goal_is_found:
			path = [index_of_new]
			parent = self._edges[index_of_new]
			while parent != 0:
				path.append(parent)
				parent = self._edges[parent]
			path.append(parent)
			path = path[::-1]
			path_as_states=[self._states[path_index] for path_index in path]
			return path_as_states

		# If the goal state has not been reached, try again
		else:
			
			print("Path not found")
			print("Trying again...")
			
			# Reset the tree
			self._states = np.array([self._states[0]])
			self._edges = {0: None}
			
			# Reset the goal flag
			self._goal_is_found = False
			
			# Try again
			return self.run()
		
			

	def sample_with_bias(self):
		"""
		Sample a point in the environment with a bias towards the goal state.
		"""
		
		# Sample a random number
		random_number = np.random.rand()

		# Sample the goal state with probability GOAL_BIAS
		if random_number < self._GOAL_BIAS:
			p_sample = self._goal_pos
		else:
			p_sample = np.array([np.random.uniform(self._environment.xbounds[0], self._environment.xbounds[1]),
							 np.random.uniform(self._environment.ybounds[0], self._environment.ybounds[1]),
							 np.random.uniform(self._environment.zbounds[0], self._environment.zbounds[1])])
		return p_sample

	def find_nearest_state(self, query_pos):
		"""
		Find the nearest state in the tree to a query position.
		"""
		
		# Extract the positions of the states
		positions = self._states[:, :3]
		
		# Calculate the Euclidean distances between the positions and the query position
		distances = np.sum((positions - query_pos)**2, axis=1)
		
		# Find the index of the state with the minimum distance
		min_index = np.argmin(distances)
		
		x_near = self._states[min_index]
		return x_near, min_index

	def integrate_forward(self, x_near, p_sample):
		"""
		Integrate the state forward from x_near towards p_sample.
		"""

		# Initialize the new state as the nearest state
		x_i = x_near
		
		# If steering is off, move towards the sample position in a straight line
		if not self._STEERING:
			return x_i + 5.0 * (p_sample-x_near[:3]) / np.linalg.norm(p_sample-x_near[:3])
		
		# Integrate the state forward in TIME_INTERVAL steps
		for _ in range(self._INTEGRATION_STEPS):
			
			# Copy the prior state
			x_prior = x_i.copy()
			
			# Update the state
			x_i = self.update_state(x_i, p_sample)
			
			# Check for being close enough to goal state
			if np.linalg.norm(x_i[:3] - self._goal_pos) <= self._GOAL_TOLERANCE:
				self._goal_is_found = True
				print("Path found")
				break
			
			# Check for collision
			if collision_check_vectorized(self._environment, x_i[:3]):
				return x_prior

			# Check for being outside the environment
			if not inside_environment(self._environment, x_i[:3]):
				return x_prior
			
		return x_i



	def update_state(self, x_near, p_sample):
		"""
		Update the state of the quadcopter from x_near to a new state that moves towards p_sample.
		Rodrigues' axis-angle rotation formula is used to update the orientation of the quadcopter.
		The formula is as follows: 
		R = I + sin(theta)K + (1 - cos(theta))K^2
		where R is the rotation matrix, I is the identity matrix, theta is the angle of rotation, and K is the skew-symmetric matrix defined
		as so: K = [[0, -k_z, k_y], [k_z, 0, -k_x], [-k_y, k_x, 0]].
		Here, k is the unit vector about which the quadcopter rotates, and theta is the angle of rotation.
		The unit vector about which the quadcopter rotates is the cross product of the prior orientation and the unit vector pointing from the
		prior position to the sample position.
		"""
		
		# Extract the prior orientation and position
		prior_attitude = x_near[3:]
		prior_position = x_near[:3]
		
		# Calculate the unit vector pointing from the prior position to the sample position
		u_1 = (p_sample - prior_position) / np.linalg.norm(p_sample - prior_position)
		
		# Calculate the axis of rotation as being perpendicular to the prior orientation and the unit vector pointing from the prior position to the sample position
		axis_vector = np.cross(prior_attitude, u_1)
		
		# If the axis of rotation is zero, the rotation matrix is the identity matrix
		if np.linalg.norm(axis_vector) <= 0.01:
			R = np.eye(3) 
		# Otherwise, calculate the rotation matrix using Rodrigues' formula
		else:
			
			# Normalize the axis of rotation
			u_2 = axis_vector / np.linalg.norm(axis_vector)
			
			# Calculate the angle of rotation
			theta = np.arccos((np.dot(prior_attitude, u_1))/(np.linalg.norm(u_1)*np.linalg.norm(prior_attitude)))
			
			# Clip the angle of rotation to the maximum steering angle
			steer_angle = np.clip(theta, -self._MAX_STEER_ANGLE, self._MAX_STEER_ANGLE)
			
			# Calculate the rotation matrix
			K = np.array([[0, -u_2[2], u_2[1]],[u_2[2], 0, -u_2[0]],[-u_2[1],u_2[0],0]])
			
			# Calculate the new attitude and position
			R = np.eye(3) + np.sin(steer_angle) * K + (1 - np.cos(steer_angle)) * np.dot(K, K)
			
		# Update the state
		new_attitude = np.dot(R, prior_attitude)
		new_position = prior_position + new_attitude * self._SPEED * self._TIME_STEP
		x_new = np.concatenate((new_position, new_attitude))
		return x_new
	
	def visualize(self, env_obj, path=[]):
		
		"""
        Visualize the RRT path planning algorithm in 3D space.
        """
        
		# Create a 3D plot
		fig = plt.figure()
		ax = fig.add_subplot(111, projection='3d')
        
		# Extract the positions of the states
		positions = self._states[:, :3]

		# Visualize the path
		X,Y,Z,A,B,C = zip(*rrt._states)
		ax.quiver(X,Y,Z, A,B,C, linewidth=1, length=5)
		px, py, pz, pa, pb, pc = zip(*path)
		ax.plot(px, py, pz, 'g-')
		ax.scatter(px, py, pz, color='lime')
		
		# Plot the start and goal positions
		ax.scatter(*self._start_pos, color='green')
		ax.scatter(*self._goal_pos, color='red')
        
		# Set the labels
		ax.set_xlabel('X')
		ax.set_ylabel('Y')
		ax.set_zlabel('Z')
		
		# Set the title
		plt.title('RRT Path Planning')
		
		# Determine the bounding box of RRT states
		rrt_states = np.array(self._states)
		min_bound = np.min(rrt_states[:, :3], axis=0) - 5  # Extra margin
		max_bound = np.max(rrt_states[:, :3], axis=0) + 5  # Extra margin

		# Function to check if boxes intersect
		def boxes_intersect(center, halfsize, min_bound, max_bound):
			return all((center - halfsize) < max_bound) and all((center + halfsize) > min_bound)

		# Visualize the relevant part of the environment
		for center, halfsize in zip(env_obj.centers, env_obj.halfsizes):
			if boxes_intersect(center, halfsize, min_bound, max_bound):
				corner = center - halfsize
				full_size = 2 * halfsize
				ax.bar3d(corner[0], corner[1], 0, full_size[0], full_size[1], full_size[2], color='r', alpha=0.5)
        
		# Show the plot
		plt.show()
	

if __name__ == "__main__":
	
	# Create an EnvironmentData object
	env_obj = EnvironmentData("../data/input/colliders.csv", 5.0)
	
	#lon, lat, alt
	start_gps = [-122.39741, 37.7911, 200]
	goal_gps = [-122.39645,  37.7931, 50]

	# Create an RRT object
	rrt = RRT(env_obj, start_gps, goal_gps, GOAL_BIAS=0.7, MAX_STEER_ANGLE_RATE=np.pi/24, TIME_STEP=0.1, 
				TIME_INTERVAL=5.0, SPEED=2.0, MAX_ITERATIONS=10000, GOAL_TOLERANCE=1.0)
		
	# Run the RRT algorithm
	path = rrt.run()
	
	# Visualize the RRT path
	rrt.visualize(env_obj, path=path)