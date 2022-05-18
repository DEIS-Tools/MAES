using System;
using System.Collections.Generic;
using Maes.ExplorationAlgorithm.RandomBallisticWalk;
using Maes.Map;
using Maes.Map.MapGen;
using Maes.Robot;
using UnityEngine;

namespace Maes
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
        public readonly string StatisticsFileName;

        public SimulationScenario(
            int seed, 
            SimulationEndCriteriaDelegate hasFinishedSim=null, 
            MapFactory mapSpawner=null, 
            RobotFactory robotSpawner=null, 
            RobotConstraints? robotConstraints=null, 
            string statisticsFileName=null
            )
        {
            Seed = seed;
            HasFinishedSim = hasFinishedSim ?? (simulation => simulation.ExplorationTracker.ExploredProportion > 0.99f || simulation.SimulatedLogicTicks > 3600 * 10);
            // Default to generating a cave map when no map generator is specified
            MapSpawner = mapSpawner ?? (generator => generator.GenerateCaveMap(new CaveMapConfig(seed)));
            RobotSpawner = robotSpawner ?? ((map, spawner) => spawner.SpawnRobotsTogether( map, seed, 2, 
                Vector2Int.zero, (robotSeed) => new RandomExplorationAlgorithm(robotSeed)));
            RobotConstraints = robotConstraints ?? new RobotConstraints();
            StatisticsFileName = statisticsFileName ?? $"statistics_{DateTime.Now.ToShortDateString().Replace('/','-')} " +
                $"_{DateTime.Now.ToShortTimeString().Replace(' ','-')}";
            
        }
        
    }
}