import json 
import pytest

@pytest.fixture(scope="session")
def config():
    with open("config.json", "r") as f:
        return json.load(f)
    
    