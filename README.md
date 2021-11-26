# Multi-Agent-Exploration-Simulator - Maes
MAES is a tool for simulating and testing exploration algorithms in a realistic continuous space environment.
Maes is visualised and physics driven using the Unity Game Engine. 
Maes was created as part of a Master's Thesis at Aalborg University, Denmark, on the subject of distributed systems. 

A video trailer for Maes can be found [here](https://youtu.be/lgUNrTfJW5g)

# Getting started
Maes requires an installed Unity editor (It is last tested with version 2020.3.18f1).
It has been tested to work on both Linux (Ubuntu 21.04), MacOS Monterey 12 and Windows 10.

Maes is already preconfigured to run with a simulation example, so it should run directly through Unity.

### Changing Simulation Setup and Scenarios
A scenario is a configuration, that can be injected into a simulation, that the simulator can then execute.
Several different sets of scenarios can be generated by the preconfigured methods inside the [ScenarioGenerator.cs](Assets/Scripts/ScenarioGenerator.cs) file.
The scenarios are then generated and queued inside the **Start()** method of the [Simulator.cs](Assets/Scripts/Simulator.cs) for later injection into a simulation.
The simulator automatically dequeues the next simulation, when the current is finished.

A scenario contains a **random seed**, a **SimulationEndCriteriaDelegate**, a **map spawner delegate**, a **robot spawner delegate** and **robot constraints**.
Additionally, a scenario contains a file name, that is used if the statistics of the given run are set to be exported. 
This can be configured inside the [GlobalSettings.cs](Assets/Scripts/GlobalSettings.cs) file.

If you create your own custom scenarios by creating a new method inside [ScenarioGenerator.cs](Assets/Scripts/ScenarioGenerator.cs), remember to call it in the [Simulator.cs](Assets/Scripts/Simulator.cs) **Start()** method.

### Creating your own algorithm
In order to implement your own algorithm, you must create a class that implements the [IExplorationAlgorithm.cs](Assets/Scripts/ExplorationAlgorithm/IExplorationAlgorithm.cs) interface.
This provides the algorithm with access to the robot controller, which in turn provides access to movement controls, slam map and all sensor data available on the agent given the constraints of the scenario.

In order to test you algorithm, make sure you configure the simulator to use a scenario that uses the algorithm.
Instructions for this can be found in [Changing Simulation Setup](#changing-simulation-setup-and-scenarios).


### Extracting Statistics
Maes supports extraction of data as csv files regarding both coverage and exploration.
Configuring statistics gathering is done in the [GlobalSettings.cs](Assets/Scripts/GlobalSettings.cs). 
Here the path for the statistics files can be changed, statistics gathering can be enabled/disabled and the interval for saving the data of a given simulation can be changed.
The csv files are created whenever a simulation finishes, if it is enabled.

The default path for the csv files is on the desktop inside a /MaesStatistics/ folder, that is created in the process.

The csv files can be used directly or processed and turned into tables using [this python script](Assets/Scripts/Statistics/main.py).

# Contributors

Philip Irming Holler - philipholler94@gmail.com

Magnus Kirkegaard Jensen - magnjensen@gmail.com

Malte Zoëga Andreasen - malte@mza.dk
