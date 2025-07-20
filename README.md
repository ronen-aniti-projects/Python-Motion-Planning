# Integrated Motion Planning Demonstration
I have engineered a Python-based motion planning pipeline for quadcopters, implementing all core algorithms (A*, PRM, RRT, and trajectory segmentation) from scratch, while incorporating unit tests, packaging, documentation, and, for demonstration and integration, a command-line interface tool. 

In brief, this motion planning pipeline software tool takes as input a user-specified configuration (`config.json`) comprising obstacle data file, start and destination GPS coordinates, and various map-building and map-searching settings, then from that configuration computes both a waypoint plan (sequence of 3D positional coordinates) and a trajectory plan (time-ordered motion profile), with all generated output data being saved to `data/output`. 

## Setup
It is recommended to use a virtual environment:

```bash 
python -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

## Usage
To launch the CLI:

```bash
pip install -e .
plan
```

## Demonstration 

### Overview

The CLI will walk you through each module interactively. You will be prompted to take the following steps:

1. Load the configuration.
2. Process the obstacle data.
3. Construct a representation of free-space. 
4. Execute A* and PRM path planning. 
5. Generate and visualize a trajectory.

#### The welcome message
On successfull launch, the CLI will show a welcome message: 
```
========== WELCOME ========== 
Welcome to a demonstration of Ronen Aniti's motion planning pipeline.
```

#### Prompt 1: Load the configuration
Initially, you will be prompted to load the configuration file.

```
--- Step 1: Load the configuration file ---
Press [D] to load configuration
Press [E] to exit
Your choice: 

```

If `config.json` loads correctly, you will see: 
```
Configuration successfully loaded.
```

#### Prompt 2: Load and visualize obstacle data
After loading the configuration, you will be prompted to load the obstacle data file (`data/input/colliders.csv`). 

```
--- Step 2: Load and visualize obstacle data ---
Press [D] to load and visualize obstacle data
Press [E] to exit
Your choice: 

```

If `colliders.csv` loads correctly, you will see summary obstacle metadata printed to the console, similar to this: 

```
Environment Data Summary:
Home Latitude: 37.79248
Home Longitude: -122.39745
Margin of Safety: 5.0
Centers: 
[[-310.2389   -439.2315     85.5     ]
 [-300.2389   -439.2315     85.5     ]
 [-290.2389   -439.2315     85.5     ]
 ...
 [ 257.8061    425.1645      1.75852 ]
 [ 293.9967    368.3391      3.557666]
 [ 281.5162    354.4156      4.999351]]
Half-sizes: 
[[10.       10.       90.5     ]
 [10.       10.       90.5     ]
 [10.       10.       90.5     ]
 ...
 [ 6.292725  6.292725  6.944791]
 [ 6.129456  6.129456  8.667319]
 [ 6.053772  6.053772  9.950246]]
Heights: 
[176.       176.       176.       ...   8.703311  12.224985  14.949597]
X Bounds: [-320.2389  609.7611]
Y Bounds: [-449.2315  480.7685]
Z Bounds: [ -5.396395 217.      ]
Lengths: [930.       930.       222.396395]

```

You will also be presented with a Matplotlib 3D plot of the obstacle landscape: 

![3D Obstacle Visualization](docs/3D_Obstacle_Visualization.png)

#### Prompt 3: 

```
--- Step 3: Construct a map of free space ---
Press [D] to demonstrate the lattice construction module
Press [E] to exit
Your choice: 

```