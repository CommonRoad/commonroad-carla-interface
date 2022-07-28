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

## Installation
to install requirement open terminal in CommonRoad-CARLA Interface and run:
```
pip install -e .
```
## Getting started
To simulate only a CommonRoad scenario in CARLA.

### 1 - Simple start

1. 3D Mode (Carla Default mode)

&nbsp; &nbsp; &nbsp; &nbsp; Set the parameters in `cr_simple_start.py` and run:
``` 
python /tutorials/cr_simple_start.py
```
2. Off-screen mode

&nbsp; &nbsp; &nbsp; &nbsp; Set the parameters in `cr_simple_start_offscreen.py` and run:

``` 
python /tutorials/cr_simple_start_offscreen.py
```
3. 2D mode can not with oneline code start.

&nbsp; &nbsp; &nbsp; &nbsp; Set the parameters in `cr_simple_start.py` and run:
```
python /tutorials/cr_simple_start.py
```
&nbsp; &nbsp; &nbsp; &nbsp; Wait until your scenario is loaded, then run:
```
python /carlacr/mode/start_2d_mode.py
```
### 2 - Start step by step

#### 3D mode (Carla default mode and off-screen mode)

1. Load configuration files and set sleep_time
```
config = set_configs()
sleep_time = config.config_carla.sleep_time
```
2. Run a CARLA server (Debian installation:)

&nbsp; &nbsp; &nbsp; &nbsp; A. 3D mode:

~~~
    with subprocess.Popen([config.config_carla.carla_path]):
        time.sleep(sleep_time)
~~~

&nbsp; &nbsp; &nbsp; &nbsp; B. Off-screen mode:

```
    with subprocess.Popen([config.config_carla.carla_path, '-RenderOffScreen']):
        time.sleep(sleep_time)
```

3. Create a CARLA client object<br/>

```
    client = carla.Client(config.carla_config.host, config.carla_config.port)
```
4. Define path of map and path of scenario 
```
    map_path = config.general.map_path
    scenario_path = config.general.scenario_path
```
5. Define name of map and name of scenario ("four_way_crossing" is an example)
```
    map_name = "four_way_crossing"
    scenario_name = "four_way_crossing_Modi"
```

6. Initialize _CarlaInterface_<br/>

~~~
    ci = CarlaInterface(open_drive_map_path, carla_client, cr_scenario_file_path)
~~~

7. Load the map in CARLA<br/>
   require all user write permission for directory: 
CarlaUE4/Content/Carla/Maps/OpenDrive/
```
    ci.load_map()
```
8. Setup CarlaInterface<br/>
```
    ci.setup_carla(hybrid_physics_mode)
```
9. Run the scenario<br/>
```
    ci.run_scenario(clean_up, carla_vehicles, carla_pedestrians)
```
## Aerial view mode (2D mode)


First run the code the same as 3D mode in `#1` Terminal.   

When the 3D mode started 
(Street from `.xodr` file showed in carla windows), 
run the `carlacr/mode/carla_2d_mode.py` in `#2` Terminal.   

#### Example for aerial view mode:

&nbsp; &nbsp; &nbsp; &nbsp; 1. Run `cr_sim_example.py` in `tutorials` in 
`#1` Terminal, and wait until your scenario is loaded 
(For example, "four_way_crossing"). 

```
    python cr_sim_example.py
```

&nbsp; &nbsp; &nbsp; &nbsp; 2. Run `carlacr/mode/carla_2d_mode.py`

```
    python carla_2d_mode.py
```

&nbsp; &nbsp; &nbsp; &nbsp; 3. You can run `cr_sim_example.py` in `tutorials` 
one more time to see the 2D view running in windows `CARLA No Rendering Mode Visualizer` from start.

![](/example_videos/carla_2D_mode_example.mp4)

## Replay mode
Watching a scenario in vehicles view with Replay Mode
    
    replaymode=CarlaReplayMode(commonroad_scenario,open_drive_map)
    replaymode.set_ego_vehicle_by_id(id)
    replaymode.saving_video(path,name,as mp4 or gif)
    replaymode.visualize()

See in example_replay_mode
## Using replay mode in command line
Example command

    python3 ./main.py ../scenarios/DEU_Test-1_1_T-1.xml ../maps/DEU_Test-1_1_T-1.xodr --veh-id 6

For further Information

    python3 ./main.py --help

## Documentation
to generate the documentation from the source, first install the dependencies with pip:
```
pip install -r docs/doc_requirements.txt
```
Afterward run:
```
cd docs && make html
```
## Additional installed packages for testing

- pytest        6.2.4
- pytest-cov    2.12.1
- coverage      5.5

To test the converter run:
pytest -v --cov=conversion.converter --cov-report html

## Structure of Commonroad-CARLA interface


![](docs/Carlacr_OnlyClass_V8.png)