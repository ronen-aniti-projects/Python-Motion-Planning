import json

import numpy as np
import pytest

from planning_algorithms.environment_data import EnvironmentData
from planning_algorithms.lattice import CubicLattice


@pytest.fixture(scope="session")
def config():
    with open("config.json", "r") as f:
        return json.load(f)
    
@pytest.fixture
def env_data(config):
    env_config = config["environment"]
    return EnvironmentData(env_config["obstacle_file"], env_config["margin_of_safety"])

@pytest.fixture
def lattice(env_data):
    return CubicLattice(
        env_data,
        center=np.array([0.0, 0.0, 10.0]),
        halfsizes=np.array([5.0, 5.0, 5.0]),
        resolution=2.0,
        connectivity="full"
    )