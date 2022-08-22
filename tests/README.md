# CommonRoad-CARLA Interface Test

## Set up run_tests
The given tests in `test_ci.py` covers most statements in `carla_mode.py` and `carla_interface.py`. 

## Notes
- To run the test, you'll need a running CARLA server end standing by. 
- To collect the coverage info, Python package `coverage` is required.
- Write permission to directory `/CarlaUE4/Content/Carla/Maps/OpenDrive/` is required by CARLA to parse the OpenDrive,
which can be done by 
    ```angular2html
    sudo chmod a+w /opt/carla-simulator/CarlaUE4/Content/Carla/Maps/OpenDrive/
    ```



## Run test
Under `tests/` directory,
```angular2html
coverage run run_tests.py
```

```angular2html
coverage report -m --omit="*/src/*,*/tests/*,*__init__*,*/opt/carla-simulator/*" || true
```
to show report in terminal (no space in omit config allowed)


```angular2html
coverage html
```
to generate html report in `htmlcov/index.html`

