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
- carla
- typer
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

    `ci = CarlaInterface(commonroad_scenario , open_drive_map, client, None)`

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
    replaymode.set_ego_vehicle_by_id(id)
    replaymode.saving_video(path,name,as mp4 or gif)
    replaymode.visualize()

![](../test_image/None.gif)

See in example_replay_mode
## Using Replay Mode in command line
Example command

    python3 ./main.py ../scenarios/DEU_Test-1_1_T-1.xml ../maps/DEU_Test-1_1_T-1.xodr --veh-id 6

For further Information

    python3 ./main.py --help

## Motion Planning Mode
Watching a scenario in vehicles view with Motion Planning Mode.This allow user to put in a motion planner, scenario and a map them have a visualization in carla.
            This API can setup map, scenario. Trigger the motion planning at the beginning of the simulation.
    
    motionplanner_mode=CarlaMotionPlannerMode(commonroad_scenario,open_drive_map,mp)
    motionplanner_mode.set_ego_vehicle_by_id(id)
    motionplanner_mode.saving_video(path,name,as mp4 or gif)
    motionplanner_mode.visualize()

See in example_motion_planning_mode
![](../test_image/DEU_Test-1_1_T-1_06_01_2022_18_53_21/None.gif)
(The black vehicle is controled by motion planner)

This mode is currently not working with command


## Documentation
to generate the documentation from the source, first install the dependencies with pip:
`pip install -r docs/doc_requirements.txt`

Afterward run:

`cd docs && make html`

## Additionally installed packages for testing

- pytest        6.2.4
- pytest-cov    2.12.1
- coverage      5.5

To test the converter run:
pytest -v --cov=conversion.converter --cov-report html

## Gitlab CI
### 1. Static Analysis
#### Goals：
Check the code using as many code inspection methods as possible to keep the code in an optimal state.
#### 1. Flake8:
`Flake8` is a Python library that wraps `PyFlakes`, `pycodestyle` and `Ned Batchelder’s McCabe script`. 
It is a great toolkit for checking your code base against coding style (PEP8), programming 
errors (like “library imported but unused” and “Undefined name”) and to check cyclomatic complexity.
#### Why:
A very common and easy to use static check package containing three check libraries.
#### 2. pycodestyle (formerly pep8):
`pycodestyle` (formerly pep8) is a tool to check your Python code against some of the style conventions in PEP 8.
- Plugin architecture: Adding new checks is easy. 
- Parseable output: Jump to error location in your editor. 
- Small: Just one Python file, requires only stdlib. You can use just the pycodestyle.py file for this purpose. 
- Comes with a comprehensive test suite.
#### Why:
`pycodestyle` is not necessary, I add it for double-check.
#### 3. Pylint:
`Pylint` is a source-code, bug and quality checker for the Python programming language. It follows the style recommended 
by PEP 8, the Python style guide. It is similar to `Pychecker` and `Pyflakes`, but includes the following features:
- Checking the length of each line
- Checking that variable names are well-formed according to the project's coding standard
- Checking that declared interfaces are truly implemented.
#### Why:
Includes a code scoring system that is very intuitive.
#### 4. MyPy:
`Mypy` is an optional static type checker for Python that aims to combine the benefits of dynamic (or "duck") typing and 
static typing. `Mypy` combines the expressive power and convenience of Python with a powerful type system and compile-time 
type checking. `Mypy` type checks standard Python programs; run them using any Python VM with basically no runtime overhead.

`Mypy` type checks programs that have type annotations conforming to `PEP 484`.
The aim is to support almost all Python language constructs in `mypy`.
#### Why:
`Mypy` is an optional static type checker and it combines the benefits of dynamic (or "duck") typing and static typing. 
It is different from the code checker described above.

#### 4. Prospector：
One of the powerful static analysis tools for analyzing Python code and displaying information about errors, potential 
issues, convention violations and complexity. It includes:
- PyLint — Code quality/Error detection/Duplicate code detection
- pep8.py — PEP8 code quality
- pep257.py — PEP27 Comment quality
- pyflakes — Error detection
- mccabe — Cyclomatic Complexity Analyser
- dodgy — secrets leak detection
- pyroma — setup.py validator
- vulture — unused code detection
#### Why:
It also contains many static check packages, which are very powerful, but are more difficult to pass and not conducive 
to step-by-step analysis. However, it can be set up through the `.prospector.yml` file and considered for later use.

### 2. Test