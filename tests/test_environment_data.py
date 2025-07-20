import numpy as np
import pytest

from planning_algorithms.environment_data import EnvironmentData

def test_home_lat_lon_properties(env_data):
    """
    Tests that the home latitude and home latitude are parsed correctly from the input file.
    """

    expected_latitude = 37.79248
    expected_longitude = -122.39745
    assert env_data.home_latitude == pytest.approx(expected_latitude)
    assert env_data.home_longitude == pytest.approx(expected_longitude)

