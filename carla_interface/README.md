# CommonRoad CarlaInterface

## Introduction

Interface for using CommonRoad together with CARLA<br/>
Entrypoint is CarlaInterface.py

## Requirements

To use the interface, CARLA (_https://carla.readthedocs.io/en/latest/start_quickstart/_) needs to be installed and running.
Also _pygame_ and _imageio_ need to be installed.<br/>
`pip install pygame imageio`

## Installation

To install the CommonRoad-CarlaInterface please run a normal pip install within the folder. 

## Getting started
To simulate only a CommonRoad scenario in CARLA without using a motion please follow those steps (see also cr_sim_example_no_mpl.py):
1. Create a CARLA client object<br/>
`client = carla.Client('localhost', 2000)`
2. Initialize _CarlaInterface_<br/>
`ci = CarlaInterface(commonroad_scenario, open_drive_map, client)`
3. Load the map in CARLA<br/>
`ci.load_map()`
4. Setup CarlaInterface<br/>
`ci.setup_carla()`
5. Run the scenario<br/>
`ci.run_scenario()`


To run the CarlaInterface together with a motion planner please take a look at https://gitlab.lrz.de/mpfav-ss21-driving-simulator/mpl-carla-example.git
