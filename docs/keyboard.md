# Keyboard Control
Use keyboard arrows (up/down/left/right) or WASD keys for control.
We have the subsequent mapping of keys to actions:

    W            : throttle
    S            : brake
    A/D          : steer left/right
    Q            : toggle reverse
    Space        : hand-brake
    P            : toggle autopilot (not implemented yet)
    L            : toggle next light type
    SHIFT + L    : toggle high beam
    Z/X          : toggle right/left blinker
    I            : toggle interior light
    O            : open/close all doors of vehicle
    T            : toggle vehicle's telemetry
    V            : toogle visualization tools
    F1           : toggle HUD
    H/?          : toggle help
    ESC          : quit

With the following code snipped you can test the keyboard control (see tutorials for complete script):
```Python
from crcarla.carla_interface import CarlaInterface
from crcarla.helper.config import CarlaParams, CustomVis, EgoPlanner

# Configure simulation and scenario settings
param = CarlaParams()
param.vehicle.vehicle_ks_state = True
param.offscreen_mode = True  # set to false if your system is powerful enough
param.vis_type = CustomVis.BIRD  # set to EGO if your system is powerful enough
param.simulation.number_vehicles = 2
param.simulation.number_walkers = 2
param.simulation.max_time_step = 120

# Initialize CARLA-Interface and start keyboard control
ci = CarlaInterface(param)
ci.external_control(EgoPlanner.KEYBOARD)

```
