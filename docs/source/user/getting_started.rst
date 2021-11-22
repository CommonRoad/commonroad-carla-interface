.. _getting_started:

===============
Getting Started
===============

This manual introduces the main functionality by means of some examples.Exact class descriptions can be found in the module descriptions.

The CommonRoad Carla Interface includes a :class:`.CarlaInterface` and its components (:class:`.CarlaPedestrianHandler`, :class:`.CarlaVehicleInterface` and so on).


You can find example in  below in "examples" folder. 


Basic example
-------------

.. code-block:: python

    import time
    import carla
    from carla_interface.CarlaInterface import CarlaInterface
    #define scenerio
	name_scenario = "DEU_Test-1_1_T-1"
	#define client
	client = carla.Client('localhost', 2000)
	ci = CarlaInterface(scenario_path + name +".xml", map_path + name_scenario +".xodr", client, None)
    	#load map
	ci.load_map()	
	time.sleep(5)  # time to move your view in carla-window
	#setup Carla
	ci.setup_carla(hybrid_physics_mode=False)
	#run Carla	
	startTime = time.time()
	ci.run_scenario(clean_up=True, carla_vehicles=0, carla_pedestrians=10)
	executionTime = (time.time() - startTime)
	print('Execution time in seconds: ' + str(executionTime))

You can run the examples after starting Carla. The Carla Interface will load map and scenerio connect to Carla server.
Note: It necessary that there are write permissions on the CarlaUE4/Content/Carla/Maps/OpenDrive/

