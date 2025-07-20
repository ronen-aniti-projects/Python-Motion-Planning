import numpy as np
from scipy.spatial import KDTree
from .lattice import CubicLattice
from .environment_data import EnvironmentData
from .utils import (
    global_to_local,
    collision_check_vectorized,
    collision_check_two_points,
    euclidean_distance,
)
from .a_star_search import astar
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


class PRM:
    """
    The Probabilistic Roadmap (PRM) organizes points according to a graph structure, where each point is a node and the connections
    between points are edges. The roadmap is constructed by randomly sampling points in the free space of the environment and connecting
    neighboring points that are not in collision with the obstacles of the environment. The roadmap can be used for graph-based search
    algorithms such as A* or Dijkstra's algorithm. The PRM class provides methods to visualize the roadmap in 3D with optional path highlighting.

    Example:

    ..  code-block:: python

            # Import EnvironmentData and PRM classes
            from environment_data import EnvironmentData
            from prm import PRM

            # Create an EnvironmentData object
            environment_data_object = EnvironmentData("obstacle_geometry.csv", 5.0)

            # Create a PRM object
            prm = PRM(environment_data_object, DENSITY=1e-4, NEIGHBORS=5)

            # Visualize the PRM
            prm.visualize(environment_data_object, path=None)

            # ... Perform path planning using A* or Dijkstra's algorithm ...

    Attributes:

            environment (EnvironmentData): an object of the EnvironmentData class
            SAMPLES (int): the number of samples to take in the free space of the environment
            NEIGHBORS (int): the number of neighbors to connect for each node in the PRM
            graph (dict): a dictionary representing the graph structure of the PRM
            points (list): a list of points in the free space of the environment
            free_space_points (list): a list of points in the free space of the environment
            free_space_points_kd_tree (scipy.spatial.KDTree): a KDTree object for efficient nearest neighbor search in the free space points
            gps_home (numpy.ndarray): a numpy array of shape (3,) containing the GPS coordinates of the home location


    """

    def __init__(self, environment_data_object, DENSITY=1e-4, NEIGHBORS=5):
        """
        Initialize the PRM object.

        Args:
                environment_data_object (EnvironmentData): an object of the EnvironmentData class
                DENSITY (float): the density of the PRM
                NEIGHBORS (int): the number of neighbors to connect for each node in the PRM

        Note:
                - The PRM is constructed by sampling points in the free space of the environment and connecting neighboring points that are not in collision with obstacles.
                - The PRM is stored as a graph with nodes and edges.
        """

        self.environment = environment_data_object
        self.SAMPLES = (
            DENSITY
            * self.environment.lengths[0]
            * self.environment.lengths[1]
            * self.environment.lengths[2]
        )
        self.NEIGHBORS = NEIGHBORS
        self.graph = {}
        self.points = []

        # To be removed later
        self.free_space_points = self.points

        self.gps_home = self.environment.gps_home

        # Take N valid samples
        self.take_n_samples(self.SAMPLES)

        # Connect nodes whose connecting segments don't intersect obstacles
        self.connect_valid_nodes()

        self.free_space_points_kd_tree = KDTree(self.points)

    def take_n_samples(self, N):
        """
        Take N samples in the free space of the environment.

        Args:
                N (int): the number of samples to take in the free space of the environment
        """

        # Sample a point one at a time, check for validity, increment n, stop when n=N
        n = 0
        while n <= N:
            x = np.random.uniform(
                self.environment.xbounds[0], self.environment.xbounds[1]
            )
            y = np.random.uniform(
                self.environment.ybounds[0], self.environment.ybounds[1]
            )
            z = np.random.uniform(
                self.environment.zbounds[0], self.environment.zbounds[1]
            )
            point = np.array([x, y, z])
            if not collision_check_vectorized(self.environment, point):
                self.points.append(point)
                n += 1

    def connect_valid_nodes(self):
        """
        Connect nodes whose connecting segments don't intersect obstacles.
        """
        points_kd_tree = KDTree(self.points)
        for index1, point1 in enumerate(self.points):
            distances, indices = points_kd_tree.query(
                point1, k=self.NEIGHBORS + 1
            )
            for distance, index2 in zip(distances[1:], indices[1:]):
                point2 = self.points[index2]
                if not collision_check_two_points(
                    self.environment, point1, point2, SPACING=1.0
                ):
                    if index1 not in self.graph:
                        self.graph[index1] = {}
                    self.graph[index1][index2] = distance

    def visualize(self, bounds, path=None):
        """
        Visualize the PRM in 3D with optional path highlighting.

        Args:

                bounds (tuple): a tuple containing the bounds of the visualization in the form (xmin, xmax, ymin, ymax, zmin, zmax)
                path (list): a list containing the indices of the points on the path to highlight

        """
        # Unpack the boundary values
        xmin, xmax, ymin, ymax, zmin, zmax = bounds

        fig = plt.figure()
        ax = fig.add_subplot(111, projection="3d")

        # Plotting obstacles within bounds
        for center, halfsize in zip(
            self.environment.centers, self.environment.halfsizes
        ):
            if (
                xmin <= center[0] <= xmax
                and ymin <= center[1] <= ymax
                and zmin <= center[2] <= zmax
            ):
                corner = center - halfsize
                full_size = 2 * halfsize
                ax.bar3d(
                    corner[0],
                    corner[1],
                    0,
                    full_size[0],
                    full_size[1],
                    full_size[2],
                    color="r",
                    alpha=0.5,
                )

        # Plotting nodes within bounds
        filtered_points = [
            point
            for point in self.points
            if xmin <= point[0] <= xmax
            and ymin <= point[1] <= ymax
            and zmin <= point[2] <= zmax
        ]
        if filtered_points:
            nodes = np.array(filtered_points)
            ax.scatter(
                nodes[:, 0], nodes[:, 1], nodes[:, 2], c="b", marker="o", s=10
            )

        # Plotting edges for nodes within bounds
        for index1, neighbors in self.graph.items():
            point1 = self.points[index1]
            if (
                xmin <= point1[0] <= xmax
                and ymin <= point1[1] <= ymax
                and zmin <= point1[2] <= zmax
            ):
                for index2 in neighbors:
                    point2 = self.points[index2]
                    if (
                        xmin <= point2[0] <= xmax
                        and ymin <= point2[1] <= ymax
                        and zmin <= point2[2] <= zmax
                    ):
                        ax.plot(
                            [point1[0], point2[0]],
                            [point1[1], point2[1]],
                            [point1[2], point2[2]],
                            "lime",
                        )

        if path is not None:
            # Highlighting edges on the path
            for i in range(len(path) - 1):
                point1 = self.points[path[i]]
                point2 = self.points[path[i + 1]]
                ax.plot(
                    [point1[0], point2[0]],
                    [point1[1], point2[1]],
                    [point1[2], point2[2]],
                    color=(0, 0, 1),
                    linewidth=2,
                    alpha=1,
                )

        ax.set_xlim([xmin, xmax])
        ax.set_ylim([ymin, ymax])
        ax.set_zlim([zmin, zmax])
        ax.set_xlabel("X axis")
        ax.set_ylabel("Y axis")
        ax.set_zlabel("Z axis")
        plt.title("PRM Visualization: Nodes, Edges, and Obstacles")

        # Subtitle the plot with the bounding position and size of the env_data object
        plt.suptitle(
            f"Bounds: ({xmin}, {xmax}, {ymin}, {ymax}, {zmin}, {zmax})"
        )

        plt.show()


if __name__ == "__main__":
    # Make an environment data object
    env_data = EnvironmentData("data/input/colliders.csv", 5.0)

    # Example usage of the search algorithms
    lattice = CubicLattice(
        env_data,
        center=np.array([0, 0, 0]),
        halfsizes=np.array([50, 50, 50]),
        resolution=15.0,
        connectivity="partial",
    )

    # Start and goal GPS coordinates
    start_gps = [-122.39745, 37.79248, 0]
    goal_gps = [-122.39645, 37.79278, 200]

    # Generate a PRM
    roadmap = PRM(env_data, DENSITY=1e-5, NEIGHBORS=5)

    # Visualize the PRM
    roadmap.visualize([-100, 100, -100, 100, 0, 200])

    # Perform A* search on the PRM
    path = astar(roadmap, start_gps, goal_gps, euclidean_distance)

    # Visualize the results
    roadmap.visualize([-100, 100, -100, 100, 0, 200], path=path)
