# CommonRoad-CARLA Interface
## Introduction
This repository contains only the current draft version of the interface itself. 
The code for converting a map from CommonRoad to OpenDRIVE is located in the CommonRoad Scenario Designer.

## Requirements
Interface for using CommonRoad together with CARLA
Entrypoint is CarlaInterface.py
- pygame
- imageio 
- moviepy
- setuptools
- numpy
- (Optional) commonroad-motion-planning-library

## Installation
to install requirement open terminal in CommonRoad-CARLA Interface and run:

`pip install -e .`

## Getting started
To simulate only a CommonRoad scenario in CARLA:
1. Run a CARLA server (Debian installation:)
    ```
   cd /opt/carla-simulator/
    ./CarlaUE4.sh
   ```
2. Create a CARLA client object<br/>
`client = carla.Client('localhost', 2000)`
3. Initialize _CarlaInterface_<br/>
    A. without MPL:

    `ci = CarlaInterface(commonroad_scenario , open_drive_map, client,None)`

    B. with MPL:

    `ci = CarlaInterface(commonroad_scenario , open_drive_map, client, MotionPlanner)`

4. Load the map in CARLA<br/>
    require all user write permissition for directory: CarlaUE4/Content/Carla/Maps/OpenDrive/

    `ci.load_map()`

5. Setup CarlaInterface<br/>
`ci.setup_carla()`
6. Run the scenario<br/>
`ci.run_scenario()`

## Replay Mode
Watching a scenario in vehicles view with Replay Mode
    
    replaymode=CarlaReplayMode(commonroad_scenario,open_drive_map)
    replaymode.set_ego_vehicle()
    replaymode.visualize()

see in example_replay_mode
## Documentation
to generate the documentation from the source, first install the dependencies with pip:

`pip install -r docs/doc_requirements.txt`

Afterward run:

`cd docs && make html`

##Additionally installed packages for testing:

- pytest        6.2.4
- pytest-cov    2.12.1
- coverage      5.5

To test the converter run:
pytest -v --cov=conversion.converter --cov-report html



