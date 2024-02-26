# CommonRoad-CARLA Interface
[![PyPI pyversions](https://img.shields.io/pypi/pyversions/commonroad-carla-interface.svg)](https://pypi.python.org/pypi/commonroad-carla-interface/)  
![Linux](https://img.shields.io/badge/Linux-FCC624?style=for-the-badge&logo=linux&logoColor=black)  
[![PyPI version fury.io](https://badge.fury.io/py/commonroad-carla-interface.svg)](https://pypi.python.org/pypi/commonroad-carla-interface/)
[![PyPI download month](https://img.shields.io/pypi/dm/commonroad-carla-interface.svg?label=PyPI%20downloads)](https://pypi.python.org/pypi/commonroad-carla-interface/) 
[![PyPI download week](https://img.shields.io/pypi/dw/commonroad-carla-interface.svg?label=PyPI%20downloads)](https://pypi.python.org/pypi/commonroad-carla-interface/)   
[![PyPI license](https://img.shields.io/pypi/l/commonroad-carla-interface.svg)](https://pypi.python.org/pypi/commonroad-carla-interface/)

The CommonRoad-CARLA Interface provides APIs to use CommonRoad-based tools together with the 3D simulator [CARLA](https://carla.org).


## Installation
### Carla
Install Carla 9.14 via as described in the 
[CARLA documentation](https://carla.readthedocs.io/en/latest/start_quickstart/#b-package-installation) 
(either use the system installation or download a release from their GitHub repository).
Also add the additional maps if you want to use them. 
You can test the successful download/usage of CARLA via running ./CarlaEU4.sh within the CARLA installation. 

### CommonRoad-CARLA Interface

#### Usage in other projects
We provide an PyPI package which can be installed with the following command
```shell
pip install commonroad-carla-interface
```

#### Development
It is recommended to use [poetry](https://python-poetry.org/) as an environment manager.
Clone the repository and install it with poetry.
```shell
git clone git@github.com:commonroad/commonroad-carla-interface.git
poetry shell
poetry install
```
We recommend to use PyCharm (Professional) as IDE.  


## Getting started
We provide [tutorial script](https://github.com/commonroad/commonroad-carla-interface/tutorials/) 
for different features we support.  
A deployed version of the documentation can be found 
[here](https://cps.pages.gitlab.lrz.de/commonroad/commonroad-carla-interface/).
You can use these tutorial scripts as the starting point.
The code for converting a map from CommonRoad to OpenDRIVE is located in 
the [CommonRoad Scenario Designer](https://commonroad.in.tum.de/tools/scenario-designer).

### Supported Features
We support five ways of interacting with CARLA:  

- **CommonRoad scenario replay**: 2D/3D visualization of CommonRoad scenarios together with solutions.  
- **Keyboard control**: Controlling an ego vehicle via keyboard in a 2D/3D visualization.
- **Scenario Generation**: Generation of CommonRoad scenarios with different configurations.
- **Motion planning**: Driving in a CARLA simulation with a CommonRoad-based motion planner.
- **Wheel control**: Controlling an ego vehicle via steering wheel and pedals in a 2D/3D visualization.


The default configuration can be found under `carlacr.helper.config` and in our documentation.    
The CARLA interface can take care of starting the CARLA server.  
The path to CARLA can either be provided manually via the config parameters or the CARLA release folder path corresponds 
to one of our default locations: /opt/carla-simulator/, /~/CARLA_0.9.14_RSS/, /~/CARLA_0.9.15/ (default paths work only 
for Ubuntu; for a complete list of default paths, see our configuration options). 


## Documentation
A deployed version of the documentation can be found 
[here](https://cps.pages.gitlab.lrz.de/commonroad/commonroad-carla-interface/).
You can also generate the documentation within your activated Poetry environment using.
```bash
poetry shell
mkdocs build
```
The documentation will be located under site, where you can open `index.html` in your browser to view it.
For updating the documentation you can also use the live preview:
```bash
poetry shell
mkdocs serve
```

## Authors
Responsible: Sebastian Maierhofer  
Contributors (in alphabetic order by last name): Linge Ai, Yassine El Alami, Rupert Bauernfeind, Vy Hong, 
Miguel Marcano, Fen Shi, Dávid Tokár Zhihao Yang

## Citation
**If you use our code for research, please consider to cite our papers:**
```
@inproceedings{Maierhofer2024a,
	author = {Maierhofer, Sebastian and  Althoff, Matthias},
	title = {CommonRoad-CARLA Interface: Bridging the Gap between Motion Planning and 3D Simulation},
	booktitle = {tbd},
	year = {2024},
	pages = {tbd},
	abstract = {Motion planning algorithms should be tested on a large, diverse, and realistic set of scenarios 
	            before deploying them in real vehicles. However, existing 3D simulators usually focus on perception 
	            and end-to-end learning, lacking specific interfaces for motion planning. We introduce the first 
	            interface for the CARLA simulator focusing on motion planning to, e.g., create configurable test 
	            scenarios and execute motion planners in interactive environments. Additionally, we introduce 
	            a converter from lanelet-based maps to OpenDRIVE, making it possible to use CommonRoad and Lanelet2 
	            maps in CARLA. Our evaluation shows that our interface is easy to use, creates new scenarios 
	            efficiently, and can successfully integrate motion planners to solve CommonRoad scenarios. We created
	            CommonRoad scenarios covering more than 300h of simulated traffic. Our tool is published as an 
	            open-source toolbox at commonroad.in.tum.de/tools/commonroad-carla-interface.},
}
```
```
@inproceedings{Maierhofer2021,
	author = {Sebastian Maierhofer, Moritz Klischat, and Matthias Althoff},
	title = {CommonRoad Scenario Designer: An Open-Source Toolbox for Map Conversion and Scenario Creation for Autonomous Vehicles},
	booktitle = {Proc. of the IEEE Int. Conf. on Intelligent Transportation Systems },
	year = {2021},
	pages = {3176-3182},
	abstract = {Maps are essential for testing autonomous driving functions. Several map and scenario formats are 
                    available. However, they are usually not compatible with each other, limiting their usability.  
                    In this paper, we address this problem using our open-source toolbox that provides map converters  
                    from different formats to the well-known CommonRoad format. Our toolbox provides converters for 
                    OpenStreetMap, Lanelet/Lanelet2, OpenDRIVE, and SUMO. Additionally, a graphical user interface is 
                    included, which allows one to efficiently create and manipulate CommonRoad maps and scenarios. 
                    We demonstrate the functionality of the toolbox by creating CommonRoad maps and scenarios based on 
                    other map formats and manually-created map data.},
}
```
