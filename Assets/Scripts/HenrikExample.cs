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
// Contributors: Rasmus Borrisholt Schmidt, Andreas Sebastian Sørensen, Thor Beregaard, Malte Z. Andreasen, Philip I. Holler and Magnus K. Jensen,
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
using Maes.ExplorationAlgorithm.RandomBallisticWalk;
using Maes.ExplorationAlgorithm.HenrikAlgo;


namespace Maes
{
    internal class HenrikExample : MonoBehaviour
    {
        private Simulator _simulator;
        /*
*/
        private void Start()
        {
            const int randomSeed = 12345;

            var constraintsDict = new Dictionary<string, RobotConstraints>();

            //var constraintsGlobalCommunication = new RobotConstraints(
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

            //var constraintsMaterials = new RobotConstraints(
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

            //var constraintsLOS = new RobotConstraints(
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

            var simulator = Simulator.GetInstance();
            var random = new System.Random(1234);
            List<int> rand_numbers = new List<int>();
            for (int i = 0; i < 100; i++)
            {
                var val = random.Next(0, 1000000);
                rand_numbers.Add(val);
            }

            var constraintName = "Global";
            var robotConstraints = constraintsDict[constraintName];

            // var buildingConfigList50 = new List<BuildingMapConfig>();
            // var buildingConfigList75 = new List<BuildingMapConfig>();
            var buildingConfigList100 = new List<BuildingMapConfig>();
            foreach (int val in rand_numbers)
            {
                // buildingConfigList50.Add(new BuildingMapConfig(val, widthInTiles: 50, heightInTiles: 50));
                // buildingConfigList75.Add(new BuildingMapConfig(val, widthInTiles: 75, heightInTiles: 75));
                buildingConfigList100.Add(new BuildingMapConfig(val, widthInTiles: 100, heightInTiles: 100));
            }

            var constraintIterator = 0;
            var mapSizes = new List<int> { 50, 75, 100 };
            var algorithms = new Dictionary<string, RobotSpawner.CreateAlgorithmDelegate>
                {
                    { "random", seed => new HenrikExplorationAlgorithm() }
                    //{ "tnf", seed => new TnfExplorationAlgorithm(1, 10, seed) }
                    // { "minotaur", seed => new MinotaurAlgorithm(robotConstraints, seed, 2) },
                    // { "greed", seed => new GreedAlgorithm() }
                };
            constraintIterator++;
            // var buildingMaps = buildingConfigList50.Union(buildingConfigList75.Union(buildingConfigList100));
            var buildingMaps = buildingConfigList100;
            foreach (var mapConfig in buildingMaps)
            {
                var robotCount = 1;
                foreach (var size in mapSizes)
                {
                    foreach (var (algorithmName, algorithm) in algorithms)
                    {

                        simulator.EnqueueScenario(new SimulationScenario(seed: 123,
                                                                         mapSpawner: generator => generator.GenerateMap(mapConfig),
                                                                         robotSpawner: (buildingConfig, spawner) => spawner.SpawnRobotsTogether(
                                                                             buildingConfig,
                                                                             seed: 123,
                                                                             numberOfRobots: robotCount,
                                                                             suggestedStartingPoint: new Vector2Int(random.Next(0, size), random.Next(0, size)),
                                                                             createAlgorithmDelegate: algorithm),
                                                                         statisticsFileName: $"{algorithmName}-seed-{mapConfig.RandomSeed}-size-{size}-comms-{constraintName}-robots-{robotCount}-SpawnTogether",
                                                                         robotConstraints: robotConstraints)
                        );

                        var spawningPosList = new List<Vector2Int>();
                        for (var amountOfSpawns = 0; amountOfSpawns < robotCount; amountOfSpawns++)
                        {
                            spawningPosList.Add(new Vector2Int(random.Next(0, size), random.Next(0, size)));
                        }

                        // simulator.EnqueueScenario(new SimulationScenario(seed: 123,
                        //                                                  mapSpawner: generator => generator.GenerateMap(mapConfig),
                        //                                                  robotSpawner: (buildingConfig, spawner) => spawner.SpawnRobotsAtPositions(
                        //                                                      collisionMap: buildingConfig,
                        //                                                      seed: 123,
                        //                                                      numberOfRobots: robotCount,
                        //                                                      spawnPositions: spawningPosList,
                        //                                                      createAlgorithmDelegate: algorithm),
                        //                                                  statisticsFileName: $"{algorithmName}-seed-{mapConfig.RandomSeed}-size-{size}-comms-{constraintName}-robots-{robotCount}-SpawnApart",
                        //                                                  robotConstraints: robotConstraints)
                        // );
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
                                                                 createAlgorithmDelegate: (seed) => new MinotaurAlgorithm(robotConstraints, seed, 2)),
                statisticsFileName: $"delete-me",
                robotConstraints: robotConstraints));

            simulator.PressPlayButton(); // Instantly enter play mode

            //simulator.GetSimulationManager().AttemptSetPlayState(SimulationPlayState.FastAsPossible);
        }

    }
}
