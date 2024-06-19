// Copyright 2024 MAES
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
// Contributors: Rasmus Borrisholt Schmidt, Andreas Sebastian Sørensen, Thor Beregaard
// 
// Original repository: https://github.com/Molitany/MAES

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

namespace Maes.ExperimentSimulations
{
    public class TimeoutTest : MonoBehaviour
    {
        private Simulator _simulator;

        private void Start()
        {
            MinosVsGreedSimulation("Global", "100");
        }

        public void MinosVsGreedSimulation(string constraintName, string amount)
        {
            var constraintsDict = new Dictionary<string, RobotConstraints>();

            constraintsDict["Global"] = new RobotConstraints(
                senseNearbyAgentsRange: 5f,
                senseNearbyAgentsBlockedByWalls: true,
                automaticallyUpdateSlam: true,
                slamUpdateIntervalInTicks: 1,
                slamSynchronizeIntervalInTicks: 10,
                slamPositionInaccuracy: 0.2f,
                distributeSlam: false,
                environmentTagReadRange: 4.0f,
                slamRayTraceRange: 7f,
                relativeMoveSpeed: 1f,
                agentRelativeSize: 0.6f,
                calculateSignalTransmissionProbability: (distanceTravelled, distanceThroughWalls) =>
                {
                    return true;
                }
            );

            constraintsDict["Material"] = new RobotConstraints(
                senseNearbyAgentsRange: 5f,
                senseNearbyAgentsBlockedByWalls: true,
                automaticallyUpdateSlam: true,
                slamUpdateIntervalInTicks: 1,
                slamSynchronizeIntervalInTicks: 10,
                slamPositionInaccuracy: 0.2f,
                distributeSlam: false,
                environmentTagReadRange: 4.0f,
                slamRayTraceRange: 7f,
                relativeMoveSpeed: 1f,
                agentRelativeSize: 0.6f,
                materialCommunication: true
            );

            constraintsDict["LOS"] = new RobotConstraints(
                senseNearbyAgentsRange: 5f,
                senseNearbyAgentsBlockedByWalls: true,
                automaticallyUpdateSlam: true,
                slamUpdateIntervalInTicks: 1,
                slamSynchronizeIntervalInTicks: 10,
                slamPositionInaccuracy: 0.2f,
                distributeSlam: false,
                environmentTagReadRange: 4.0f,
                slamRayTraceRange: 7f,
                relativeMoveSpeed: 1f,
                agentRelativeSize: 0.6f,
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
            var algorithms = new Dictionary<string, RobotSpawner.CreateAlgorithmDelegate>
                {
                    { "tnf", seed => new TnfExplorationAlgorithm(1, 10, seed) },
                    { "minotaur", seed => new MinotaurAlgorithm(constraintsDict[constraintName], seed, 2) },
                    { "greed", seed => new GreedAlgorithm() }
                };
            var simulator = Simulator.GetInstance();
            var random = new System.Random(1234);
            List<int> rand_numbers = new List<int>();
            for (int i = 0; i < 100; i++)
            {
                var val = random.Next(0, 1000000);
                rand_numbers.Add(val);
            }

            var buildingConfigList = new List<BuildingMapConfig>();
            switch (amount)
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

            var previousSimulations = Directory.GetFiles(Path.GetFullPath("./" + GlobalSettings.StatisticsOutPutPath));
            var algorithmName = "minotaur";
            foreach (var mapConfig in buildingConfigList)
            {
                for (var amountOfRobots = 1; amountOfRobots <= 9; amountOfRobots += 2)
                {
                    var robotCount = amountOfRobots;

                    var regex = new Regex($@"{algorithmName}-seed-{mapConfig.RandomSeed}-mapConfig\.HeightInTiles-{mapConfig.HeightInTiles}-comms-{constraintName}-robots-{robotCount}-SpawnTogether_.*\.csv");
                    if (robotCount == 5 && mapConfig.RandomSeed == 585462)
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
                            robotConstraints: constraintsDict[constraintName]));
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
                    if (false)
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
                                        robotConstraints: constraintsDict[constraintName]));
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
