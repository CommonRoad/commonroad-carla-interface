# CommonRoad-CARLA Interface Test

## Set up run_tests
If you want to turn on rendering for Carla please delete the option -RenderOffScreen in run_tests
If you want to turn on GUI please uncomment the GUI comment in each test

## Motion Planner Mode
To set a test for this mode requires a meaning for to import Motion planner library module. Then set it up with motion_planner config

## Uncoveraged  
There still requires a test case for pedestrian.
The other interfaces can be test through Replay Mode amd Traffic Generation mode. It would be ideal to have tests for these but it is quite hard to set up.

## Run test
Under test directory,
> coverage run run_tests.py
> 
> coverage report -m --omit="*/src/*,*/tests/*" || true     # show report in terminal
> 
> coverage html     # generate html report under directory htmlcov/index.html

