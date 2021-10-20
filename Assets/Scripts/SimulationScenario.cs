using System.Collections.Generic;
using Dora.MapGeneration;
using Dora.Robot;

namespace Dora
{
    
    // A function that generates, initializes and returns a world map
    public delegate SimulationMap<bool> MapFactory(MapGenerator generator);
    // A function that spawns and returns a group of robots
    public delegate List<MonaRobot> RobotFactory(SimulationMap<bool> map, RobotSpawner spawner);
    
    // A function that returns true if the given simulation has been completed
    public delegate bool SimulationEndCriteriaDelegate(Simulation simulation);
    
    // Contains all information needed for simulating a single simulation scenario
    // (One map, one type of robots)
    public class SimulationScenario
    {
        private readonly int Seed;
        public readonly SimulationEndCriteriaDelegate HasFinishedSim; 
        
        public readonly MapFactory MapSpawner;
        public readonly RobotFactory RobotSpawner;
        public readonly RobotConstraints RobotConstraints; 

        public SimulationScenario(int seed, SimulationEndCriteriaDelegate hasFinishedSim, MapFactory mapSpawner, RobotFactory robotSpawner, RobotConstraints robotConstraints)
        {
            Seed = seed;
            HasFinishedSim = hasFinishedSim;
            MapSpawner = mapSpawner;
            RobotSpawner = robotSpawner;
            RobotConstraints = robotConstraints;
        }
    }
}