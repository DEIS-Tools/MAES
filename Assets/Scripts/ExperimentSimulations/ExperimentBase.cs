// Copyright 2022 MAES
// 
// This file is part of MAES
// 
// MAES is free software: you can redistribute it and/or modify it under
// the terms of the GNU General Public License as published by the
// Free Software Foundation, either version 3 of the License, or (at your option)
// any later version.
// 
// MAES is distributed in the hope that it will be useful, but WITHOUT
// ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
// or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
// Public License for more details.
// 
// You should have received a copy of the GNU General Public License along
// with MAES. If not, see http://www.gnu.org/licenses/.
// 
// Contributors: Malte Z. Andreasen, Philip I. Holler and Magnus K. Jensen
// 
// Original repository: https://github.com/MalteZA/MAES

using System;
using Maes.ExplorationAlgorithm.Minotaur;
using System.Collections;
using Maes.ExplorationAlgorithm.TheNextFrontier;
using Maes.Map;
using Maes.Map.MapGen;
using Maes.Robot;
using Maes.Utilities.Files;
using UnityEngine;
using Maes.Robot;
using Maes.ExplorationAlgorithm.Movement;
using System.Collections.Generic;
using Maes.UI;
using UnityEditor;
using System.Linq;
using Maes.ExplorationAlgorithm.Greed;
using System.Text.RegularExpressions;
using System.IO;
using Maes.ExplorationAlgorithm.Voronoi;

namespace Maes.ExperimentSimulations
{
    public class ExperimentBase : MonoBehaviour
    {
        private Simulator _simulator;
        
        /// <summary>
        /// This class will run mostly all configurations, it is written fast and loose.
        /// </summary>
        /// <param name="mapType"></param>
        /// <param name="algorithmName"></param>
        /// <param name="constraintName"></param>
        /// <param name="mapSize"></param>
        public void RunSimulation(string mapType, string algorithmName, string constraintName, string mapSize, int mapIterations, int? desiredSeed = null, int? desiredRobots = null)
        {
            var constraintsDict = new Dictionary<string, RobotConstraints>();

            constraintsDict["Global"] = new RobotConstraints(
                slamUpdateIntervalInTicks: 1,
                slamRayTraceRange: 7f,
                calculateSignalTransmissionProbability: (distanceTravelled, distanceThroughWalls) =>
                {
                    return true;
                }
            );

            constraintsDict["Material"] = new RobotConstraints(
                slamUpdateIntervalInTicks: 1,
                slamRayTraceRange: 7f,
                materialCommunication: true
            );

            constraintsDict["LOS"] = new RobotConstraints(
                slamUpdateIntervalInTicks: 1,
                slamRayTraceRange: 7f,
                calculateSignalTransmissionProbability: (distanceTravelled, distanceThroughWalls) =>
                {
                    // Blocked by walls
                    if (distanceThroughWalls > 0)
                    {
                        return false;
                    }
                    return true;
                }
            );

            constraintsDict["None"] = new RobotConstraints(
                slamUpdateIntervalInTicks: 1,
                slamRayTraceRange: 7f,
                calculateSignalTransmissionProbability: (distanceTravelled, distanceThroughWalls) =>
                {
                    return false;
                }
            );
            var algorithms = new Dictionary<string, RobotSpawner.CreateAlgorithmDelegate>
                {
                    { "tnf", seed => new TnfExplorationAlgorithm(1, 10, seed) },
                    { "minotaur", seed => new MinotaurAlgorithm(constraintsDict[constraintName], seed, 2) },
                    { "greed", seed => new GreedAlgorithm() },
                    { "voronoi", seed => new VoronoiExplorationAlgorithm(seed, constraintsDict[constraintName], constraintsDict[constraintName].SlamRayTraceRange-1) }
                };
            var simulator = Simulator.GetInstance();
            var random = new System.Random(1234);
            List<int> rand_numbers = new List<int>();
            for (int i = 0; i < mapIterations; i++)
            {
                var val = random.Next(0, 1000000);
                rand_numbers.Add(val);
            }



            var buildingConfigList = new List<BuildingMapConfig>();
            var caveConfigList = new List<CaveMapConfig>();
            if (mapType == "building")
            {
                switch (mapSize)
                {
                    case "50":
                        foreach (var val in rand_numbers)
                        {
                            buildingConfigList.Add(new BuildingMapConfig(val, widthInTiles: 50, heightInTiles: 50));
                        }
                        break;
                    case "75":
                        foreach (var val in rand_numbers)
                        {
                            buildingConfigList.Add(new BuildingMapConfig(val, widthInTiles: 75, heightInTiles: 75));
                        }
                        break;
                    case "100":
                        foreach (var val in rand_numbers)
                        {
                            buildingConfigList.Add(new BuildingMapConfig(val, widthInTiles: 100, heightInTiles: 100));
                        }
                        break;
                    default:
                        break;
                }
            }
            else
            {
                switch (mapSize)
                {
                    case "50":
                        foreach (var val in rand_numbers)
                        {
                            caveConfigList.Add(new CaveMapConfig(val, widthInTiles: 50, heightInTiles: 50));
                        }
                        break;
                    case "75":
                        foreach (var val in rand_numbers)
                        {
                            caveConfigList.Add(new CaveMapConfig(val, widthInTiles: 75, heightInTiles: 75));
                        }
                        break;
                    case "100":
                        foreach (var val in rand_numbers)
                        {
                            caveConfigList.Add(new CaveMapConfig(val, widthInTiles: 100, heightInTiles: 100));
                        }
                        break;
                    default:
                        break;
                }

            }

            var previousSimulations = Directory.GetFiles(Path.GetFullPath("./" + GlobalSettings.StatisticsOutPutPath));
            if (mapType == "building")
            {
                foreach (var mapConfig in buildingConfigList)
                {
                    for (var amountOfRobots = 1; amountOfRobots <= 9; amountOfRobots += 2)
                    {
                        var robotCount = amountOfRobots;
                        var regex = new Regex($@"{algorithmName}-seed-{mapConfig.RandomSeed}-mapConfig\.HeightInTiles-{mapConfig.HeightInTiles}-comms-{constraintName}-robots-{robotCount}-SpawnTogether_.*\.csv");
                        if (((desiredSeed.HasValue && desiredSeed.Value == mapConfig.RandomSeed) && (desiredRobots.HasValue && desiredRobots == robotCount)) || (!desiredSeed.HasValue && !previousSimulations.Any(simulation => regex.IsMatch(simulation))))
                        {
                            simulator.EnqueueScenario(new SimulationScenario(seed: 123,
                                                                            mapSpawner: generator => generator.GenerateMap(mapConfig),
                                                                            robotSpawner: (buildingConfig, spawner) => spawner.SpawnRobotsTogether(
                                                                                buildingConfig,
                                                                                seed: 123,
                                                                                numberOfRobots: robotCount,
                                                                                suggestedStartingPoint: new Vector2Int(random.Next(-mapConfig.HeightInTiles / 2, mapConfig.HeightInTiles / 2), random.Next(-mapConfig.HeightInTiles / 2, mapConfig.HeightInTiles / 2)),
                                                                                createAlgorithmDelegate: algorithms[algorithmName]),
                                                                            statisticsFileName: $"{algorithmName}-seed-{mapConfig.RandomSeed}-mapConfig.HeightInTiles-{mapConfig.HeightInTiles}-comms-{constraintName}-robots-{robotCount}-SpawnTogether",
                                                                            robotConstraints: constraintsDict[constraintName])
                            );
                        }
                        else
                        {
                            new SimulationScenario(seed: 123,
                                                mapSpawner: generator => generator.GenerateMap(mapConfig),
                                                robotSpawner: (buildingConfig, spawner) => spawner.SpawnRobotsTogether(
                                                    buildingConfig,
                                                    seed: 123,
                                                    numberOfRobots: robotCount,
                                                    suggestedStartingPoint: new Vector2Int(random.Next(-mapConfig.HeightInTiles / 2, mapConfig.HeightInTiles / 2), random.Next(-mapConfig.HeightInTiles / 2, mapConfig.HeightInTiles / 2)),
                                                    createAlgorithmDelegate: algorithms[algorithmName]),
                                                statisticsFileName: $"{algorithmName}-seed-{mapConfig.RandomSeed}-mapConfig.HeightInTiles-{mapConfig.HeightInTiles}-comms-{constraintName}-robots-{robotCount}-SpawnTogether",
                                                robotConstraints: constraintsDict[constraintName]);
                        }

                        var spawningPosHashSet = new HashSet<Vector2Int>();
                        while (spawningPosHashSet.Count < amountOfRobots)
                        {
                            spawningPosHashSet.Add(new Vector2Int(random.Next(-mapConfig.HeightInTiles / 2, mapConfig.HeightInTiles / 2), random.Next(-mapConfig.HeightInTiles / 2, mapConfig.HeightInTiles / 2)));
                        }


                        regex = new Regex($@"{algorithmName}-seed-{mapConfig.RandomSeed}-mapConfig\.HeightInTiles-{mapConfig.HeightInTiles}-comms-{constraintName}-robots-{robotCount}-SpawnApart_.*\.csv");
                        if (((desiredSeed.HasValue && desiredSeed.Value == mapConfig.RandomSeed) && (desiredRobots.HasValue && desiredRobots == robotCount)) || (!desiredSeed.HasValue && !previousSimulations.Any(simulation => regex.IsMatch(simulation))))
                        {
                            simulator.EnqueueScenario(new SimulationScenario(seed: 123,
                                                                            mapSpawner: generator => generator.GenerateMap(mapConfig),
                                                                            robotSpawner: (buildingConfig, spawner) => spawner.SpawnRobotsAtPositions(
                                                                                collisionMap: buildingConfig,
                                                                                seed: 123,
                                                                                numberOfRobots: robotCount,
                                                                                spawnPositions: spawningPosHashSet.ToList(),
                                                                                createAlgorithmDelegate: algorithms[algorithmName]),
                                                                            statisticsFileName: $"{algorithmName}-seed-{mapConfig.RandomSeed}-mapConfig.HeightInTiles-{mapConfig.HeightInTiles}-comms-{constraintName}-robots-{robotCount}-SpawnApart",
                                                                            robotConstraints: constraintsDict[constraintName])
                            );
                        }
                        else
                        {
                            new SimulationScenario(seed: 123,
                                                mapSpawner: generator => generator.GenerateMap(mapConfig),
                                                robotSpawner: (buildingConfig, spawner) => spawner.SpawnRobotsAtPositions(
                                                    collisionMap: buildingConfig,
                                                    seed: 123,
                                                    numberOfRobots: robotCount,
                                                    spawnPositions: spawningPosHashSet.ToList(),
                                                    createAlgorithmDelegate: algorithms[algorithmName]),
                                                statisticsFileName: $"{algorithmName}-seed-{mapConfig.RandomSeed}-mapConfig.HeightInTiles-{mapConfig.HeightInTiles}-comms-{constraintName}-robots-{robotCount}-SpawnApart",
                                                robotConstraints: constraintsDict[constraintName]);
                        }
                    }
                }
            }
            else
            {
                foreach (var mapConfig in caveConfigList)
                {
                    for (var amountOfRobots = 1; amountOfRobots <= 9; amountOfRobots += 2)
                    {
                        var robotCount = amountOfRobots;
                        var regex = new Regex($@"{mapType}-{algorithmName}-seed-{mapConfig.RandomSeed}-mapConfig.HeightInTiles-{mapConfig.HeightInTiles}-comms-{constraintName}-robots-{robotCount}-SpawnTogether_.*\.csv");
                        if (!previousSimulations.Any(simulation => regex.IsMatch(simulation)))
                        {
                            simulator.EnqueueScenario(new SimulationScenario(seed: 123,
                                                                            mapSpawner: generator => generator.GenerateMap(mapConfig),
                                                                            robotSpawner: (buildingConfig, spawner) => spawner.SpawnRobotsTogether(
                                                                                buildingConfig,
                                                                                seed: 123,
                                                                                numberOfRobots: robotCount,
                                                                                suggestedStartingPoint: new Vector2Int(random.Next(-mapConfig.HeightInTiles / 2, mapConfig.HeightInTiles / 2), random.Next(-mapConfig.HeightInTiles / 2, mapConfig.HeightInTiles / 2)),
                                                                                createAlgorithmDelegate: algorithms[algorithmName]),
                                                                            statisticsFileName: $"{mapType}-{algorithmName}-seed-{mapConfig.RandomSeed}-mapConfig.HeightInTiles-{mapConfig.HeightInTiles}-comms-{constraintName}-robots-{robotCount}-SpawnTogether",
                                                                            robotConstraints: constraintsDict[constraintName])
                            );
                        }
                        else
                        {
                            new SimulationScenario(seed: 123,
                                                mapSpawner: generator => generator.GenerateMap(mapConfig),
                                                robotSpawner: (buildingConfig, spawner) => spawner.SpawnRobotsTogether(
                                                    buildingConfig,
                                                    seed: 123,
                                                    numberOfRobots: robotCount,
                                                    suggestedStartingPoint: new Vector2Int(random.Next(-mapConfig.HeightInTiles / 2, mapConfig.HeightInTiles / 2), random.Next(-mapConfig.HeightInTiles / 2, mapConfig.HeightInTiles / 2)),
                                                    createAlgorithmDelegate: algorithms[algorithmName]),
                                                statisticsFileName: $"{mapType}-{algorithmName}-seed-{mapConfig.RandomSeed}-mapConfig.HeightInTiles-{mapConfig.HeightInTiles}-comms-{constraintName}-robots-{robotCount}-SpawnTogether",
                                                robotConstraints: constraintsDict[constraintName]);
                        }

                        var spawningPosHashSet = new HashSet<Vector2Int>();
                        while (spawningPosHashSet.Count < amountOfRobots)
                        {
                            spawningPosHashSet.Add(new Vector2Int(random.Next(-mapConfig.HeightInTiles / 2, mapConfig.HeightInTiles / 2), random.Next(-mapConfig.HeightInTiles / 2, mapConfig.HeightInTiles / 2)));
                        }


                        regex = new Regex($@"{mapType}-{algorithmName}-seed-{mapConfig.RandomSeed}-mapConfig.HeightInTiles-{mapConfig.HeightInTiles}-comms-{constraintName}-robots-{robotCount}-SpawnApart_.*\.csv");
                        if (!previousSimulations.Any(simulation => regex.IsMatch(simulation)))
                        {
                            simulator.EnqueueScenario(new SimulationScenario(seed: 123,
                                                                            mapSpawner: generator => generator.GenerateMap(mapConfig),
                                                                            robotSpawner: (buildingConfig, spawner) => spawner.SpawnRobotsAtPositions(
                                                                                collisionMap: buildingConfig,
                                                                                seed: 123,
                                                                                numberOfRobots: robotCount,
                                                                                spawnPositions: spawningPosHashSet.ToList(),
                                                                                createAlgorithmDelegate: algorithms[algorithmName]),
                                                                            statisticsFileName: $"{mapType}-{algorithmName}-seed-{mapConfig.RandomSeed}-mapConfig.HeightInTiles-{mapConfig.HeightInTiles}-comms-{constraintName}-robots-{robotCount}-SpawnApart",
                                                                            robotConstraints: constraintsDict[constraintName])
                            );
                        }
                        else
                        {
                            new SimulationScenario(seed: 123,
                                                mapSpawner: generator => generator.GenerateMap(mapConfig),
                                                robotSpawner: (buildingConfig, spawner) => spawner.SpawnRobotsAtPositions(
                                                    collisionMap: buildingConfig,
                                                    seed: 123,
                                                    numberOfRobots: robotCount,
                                                    spawnPositions: spawningPosHashSet.ToList(),
                                                    createAlgorithmDelegate: algorithms[algorithmName]),
                                                statisticsFileName: $"{mapType}-{algorithmName}-seed-{mapConfig.RandomSeed}-mapConfig.HeightInTiles-{mapConfig.HeightInTiles}-comms-{constraintName}-robots-{robotCount}-SpawnApart",
                                                robotConstraints: constraintsDict[constraintName]);
                        }
                    }
                }
            }
            //Just code to make sure we don't get too many maps of the last one in the experiment
            var dumpMap = new BuildingMapConfig(-1, widthInTiles: 50, heightInTiles: 50);
            simulator.EnqueueScenario(new SimulationScenario(seed: 123,
                mapSpawner: generator => generator.GenerateMap(dumpMap),
                robotSpawner: (buildingConfig, spawner) => spawner.SpawnRobotsTogether(
                                                                 buildingConfig,
                                                                 seed: 123,
                                                                 numberOfRobots: 5,
                                                                 suggestedStartingPoint: Vector2Int.zero,
                                                                 createAlgorithmDelegate: (seed) => new MinotaurAlgorithm(constraintsDict[constraintName], seed, 2)),
                statisticsFileName: $"delete-me",
                robotConstraints: constraintsDict[constraintName]));

            simulator.PressPlayButton(); // Instantly enter play mode

            //simulator.GetSimulationManager().AttemptSetPlayState(SimulationPlayState.FastAsPossible);
        }
    }
}
