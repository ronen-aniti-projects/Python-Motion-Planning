"""
This module comprises functionality to parse and process obstacle geometry data, \
ensuring it's able to be read by the other modules of this package.
"""

import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial import KDTree


class EnvironmentData:
    """
    Class to store and visualize environment data for path planning. The environment is modeled as a set of obstacles in 3D space.
    This class organized the environment data into properties and methods for easy access by other elements of the path planning system.
    Summary and visualization methods are provided to help the programmer understand the environment data.

    Note:
        The environment data consists of obstacle geometry data read from a file. The file should contain the following data:

        - First row: Geodetic position of the home location (latitude, longitude, altitude).
        - Second row: Header row with column names.
        - Subsequent rows: Obstacle geometry data (center x, center y, center z, half-size x, half-size y, half-size z).
        - The obstacle geometry data should be in the form of a comma-separated values (CSV) file.
        - The obstacle geometry data should be in the local NED (North-East-Down) frame of reference.
        - The margin of safety is added to the obstacle half-sizes to account for the size of the planning vehicle.
        - The environment data is visualized in 3D using matplotlib.
        - The environment data is stored in a spatial data structure for fast nearest neighbor obstacle lookups.
        - The environment data is summarized using a method that prints the home location, margin of safety, and obstacle data.
    """

    def __init__(self, obstacle_geometry_file, margin_of_safety):
        """
        Constructor for the EnvironmentData class.

        Args:
            obstacle_geometry_file (str): The path to the obstacle geometry file.
            margin_of_safety (float): The margin of safety added to the obstacle half-sizes.
        """

        # Reading obstacle geometry from a file
        obstacle_geometry_as_array = np.genfromtxt(
            obstacle_geometry_file, delimiter=",", skip_header=2
        )

        # Assuming 'csv_file' is meant to be 'obstacle_geometry_file' for reading geodetic positions
        geodetic_position_as_array = np.genfromtxt(
            obstacle_geometry_file, delimiter=",", dtype="str", max_rows=1
        )

        # Extracting home longitude and latitude
        self._home_latitude = float(geodetic_position_as_array[0].split()[1])
        self._home_longitude = float(geodetic_position_as_array[1].split()[1])
        self._home_altitude = 0.0
        self._gps_home = np.array(
            [self._home_longitude, self._home_latitude, self._home_altitude]
        )

        # Setting margin of safety and calculating centers, half-sizes, and heights
        self._margin_of_safety = margin_of_safety
        self._centers = obstacle_geometry_as_array[:, :3]
        self._halfsizes = (
            obstacle_geometry_as_array[:, 3:] + self._margin_of_safety
        )
        self._heights = self._centers[:, 2] + self._halfsizes[:, 2]

        # Calculating bounds and lengths
        xmin = np.min(self._centers[:, 0] - self._halfsizes[:, 0])
        xmax = np.max(self._centers[:, 0] + self._halfsizes[:, 0])
        ymin = np.min(self._centers[:, 1] - self._halfsizes[:, 1])
        ymax = np.max(self._centers[:, 1] + self._halfsizes[:, 1])
        zmin = np.min(self._centers[:, 2] - self._halfsizes[:, 2])
        zmax = np.max(self._centers[:, 2] + self._halfsizes[:, 2])
        self._xbounds = np.array([xmin, xmax])
        self._ybounds = np.array([ymin, ymax])
        self._zbounds = np.array([zmin, zmax])
        self._lengths = np.array([xmax - xmin, ymax - ymin, zmax - zmin])

        # Incorporating spatial data structure for fast nearest neighbor obstacle lookups
        self._ground_centers = KDTree(self._centers[:, :2])

    # Property methods to access private attributes
    @property
    def home_latitude(self):
        """
        The home latiiude of the environment. Type: float.
        """
        return self._home_latitude

    @property
    def home_longitude(self):
        """
        The home longitude of the environment. Type: float.
        """
        return self._home_longitude

    @property
    def gps_home(self):
        """
        The GPS coordinates of the home location. Type: numpy.ndarray.
        """
        return self._gps_home

    @property
    def margin_of_safety(self):
        """
        The margin of safety added to the obstacle half-sizes. Type: float.

        """
        return self._margin_of_safety

    @property
    def centers(self):
        """
        The centers of the obstacles in the environment. Type: numpy.ndarray.
        """
        return self._centers

    @property
    def halfsizes(self):
        """
        The half-sizes of the obstacles in the environment. Type: numpy.ndarray.
        """
        return self._halfsizes

    @property
    def heights(self):
        """
        The heights of the obstacles in the environment. Type: numpy.ndarray.
        """
        return self._heights

    @property
    def xbounds(self):
        """
        The x-bounds of the environment. Type: numpy.ndarray.
        """

        return self._xbounds

    @property
    def ybounds(self):
        """
        The y-bounds of the environment. Type: numpy.ndarray.
        """
        return self._ybounds

    @property
    def zbounds(self):
        """
        The z-bounds of the environment. Type: numpy.ndarray.
        """
        return self._zbounds

    @property
    def lengths(self):
        """
        The lengths of the environment. Type: numpy.ndarray.
        """
        return self._lengths

    @property
    def ground_centers(self):
        """
        The KDTree of the ground centers of the obstacles in the environment. Type: scipy.spatial.KDTree.
        """
        return self._ground_centers

    def summary(self):
        """
        Method to print a summary of the environment data.
        """
        print("Environment Data Summary:")
        print(f"Home Latitude: {self.home_latitude}")
        print(f"Home Longitude: {self.home_longitude}")
        print(f"Margin of Safety: {self.margin_of_safety}")
        print(f"Centers: \n{self.centers}")
        print(f"Half-sizes: \n{self.halfsizes}")
        print(f"Heights: \n{self.heights}")
        print(f"X Bounds: {self.xbounds}")
        print(f"Y Bounds: {self.ybounds}")
        print(f"Z Bounds: {self.zbounds}")
        print(f"Lengths: {self.lengths}")

    def visualize(self):
        """
        Method to visualize the environment data in 3D.
        """
        fig = plt.figure()
        ax = fig.add_subplot(111, projection="3d")
        for center, halfsize in zip(self.centers, self.halfsizes):
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
        ax.set_xlabel("X axis")
        ax.set_ylabel("Y axis")
        ax.set_zlabel("Z axis")
        plt.title("3D Obstacle Visualization")
        plt.show()

    def add_obstacles(centers, halfsizes):
        """
        Method to add obstacles to the environment data.

        Args:
            centers (numpy.ndarray): a numpy array of shape (n, 3) containing the centers of the obstacles to add
            halfsizes (numpy.ndarray): a numpy array of shape (n, 3) containing the half-sizes of the obstacles to add


        Note:
            This could be useful for adding dynamic obstacles to the environment data after initialization.
        """
        pass


if __name__ == "__main__":
    # Constructs an EnvironmentData object, parsing the obstacle data from 'colliders.csv' and setting the margin of safety to 5.0
    enivronment_data = EnvironmentData("data/input/colliders.csv", 5.0)

    # Prints a summary of the environment data key features
    enivronment_data.summary()

    # Visualizes the environment data in 3D
    enivronment_data.visualize()
