# CommonRoad-CARLA Interface
## Introduction
The CommonRoad-CARLA Interface provides APIs to use CommonRoad-based tools together with the 3D simulator CARLA 
(version 0.0.14).   
The code for converting a map from CommonRoad to OpenDRIVE is located in the CommonRoad Scenario Designer.

## Installation
The usage of [Poetry](https://python-poetry.org/) is recommended. 
Poetry can be installed using:
```bash
curl -sSL https://install.python-poetry.org | python3 -
```
Please follow the prompted instructions and add poetry to your system paths so that you can use the _poetry_ command.

Create a new Python environment:
```bash
poetry shell
poetry install --with tests,docs,tutorials
```
We recommend to use PyCharm (Professional) as IDE.  
The path to CARLA can either be provided manually via the config parameters or the CARLA release folder path corresponds 
to one of our default locations: /opt/carla-simulator/, /~/CARLA_0.9.14_RSS/, /~/CARLA_0.9.14/ (default paths work only 
for Ubuntu).

### Documentation
You can generate the documentation within your activated Python environment using
```bash
cd docs/source && sphinx-build -b html . ../public
```
The documentation will be located under docs/public. 


## Getting started
We support five ways of interacting with CARLA:
- CommonRoad scenario replay: 2D/3D visualization of CommonRoad scenarios together with solutions.  
- Keyboard control: Controlling an ego vehicle via keyboard in a 2D/3D visualization.
- Wheel control (not yet implemented): Controlling an ego vehicle via steering wheel and pedals in a 2D/3D 
visualization.
- Scenario Generation: Generation of CommonRoad scenarios with different configurations.
- Motion planning (wip): Driving in a CARLA simulation with a CommonRoad-based motion planner.

We support CARLA's synchronous and asynchronous mode as well as its offscreen mode.  
For the offscreen mode, we support a 2D birds-eye visualization.  
The default configuration can be found under carlacr/helper/config.py.  
The CARLA interface can take care about starting the CARLA server.

You can find example scripts showing how to use the CommonRoad-CARLA Interface within the folder tutorials/.











