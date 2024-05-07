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

namespace Maes
{
    internal class ExampleProgram : MonoBehaviour
    {
        private Simulator _simulator;
        private void Start()
        {
            const int randomSeed = 12; // 948778

            var constraintsGlobalCommunication = new RobotConstraints(
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

            var constraintsMaterials = new RobotConstraints(
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

            var constraints = new RobotConstraints(
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
                    // Max distance 15.0f
                    else if (15.0f < distanceTravelled)
                    {
                        return false;
                    }

                    return true;
                }
            );

            var simulator = Simulator.GetInstance();
            RobotSpawner.CreateAlgorithmDelegate tnf = seed => new TnfExplorationAlgorithm(1, 10, seed);
            RobotSpawner.CreateAlgorithmDelegate minos = seed => new MinotaurAlgorithm(constraints, seed, 6);


            var random = new System.Random(1234);

            List<int> rand_numbers = new List<int>();

            for (int i = 0; i < 10; i++)
            {
                var val = random.Next(0, 1000000);
                rand_numbers.Add(val);
                Debug.Log(val);
            }

            //var scenarios = ScenarioGenerator.GenerateTnfScenarios();
            //foreach (SimulationScenario sce in scenarios){
            //    simulator.EnqueueScenario(sce);
            //}
            var buildingConfigList50 = new List<BuildingMapConfig>();
            var buildingConfigList75 = new List<BuildingMapConfig>();
            var buildingConfigList100 = new List<BuildingMapConfig>();
            foreach (int val in rand_numbers)
            {
                buildingConfigList50.Add(new BuildingMapConfig(val, widthInTiles: 50, heightInTiles: 50, doorWidth: 6, minRoomSideLength: 11));
                //buildingConfigList75.Add(new BuildingMapConfig(val, widthInTiles: 75, heightInTiles: 75));
                //buildingConfigList100.Add(new BuildingMapConfig(val, widthInTiles: 100, heightInTiles: 100));
            }

            foreach (var mapConfig in buildingConfigList50)
            {
                //var map = PgmMapFileLoader.LoadMapFromFileIfPresent("doorway_corner.pgm");
                simulator.EnqueueScenario(new SimulationScenario(seed: 123, 
                    mapSpawner: generator => generator.GenerateMap(mapConfig),
                    robotSpawner: (buildingConfig, spawner) => spawner.SpawnRobotsTogether(
                                                                     buildingConfig,
                                                                     seed: 123,
                                                                     numberOfRobots: 5,
                                                                     suggestedStartingPoint: Vector2Int.zero,
                                                                     createAlgorithmDelegate: minos),
                    statisticsFileName: $"doorway6-{mapConfig.RandomSeed}-",
                    robotConstraints: constraints)
                );

                //var scenario = new SimulationScenario(
                //    seed: randomSeed,
                //    mapSpawner: generator => generator.GenerateMap(buildingConfig),
                //    robotConstraints: constraints,
                //    robotSpawner: (map, robotSpawner) => robotSpawner.SpawnRobotsTogether(
                //        map,
                //        123,
                //        5,
                //        new Vector2Int(0, 0),
                //        (seed) => new TnfExplorationAlgorithm(1, 10, 123)
                //    ));
                // Get/instantiate simulation prefab

                //var buildingConfig = new CaveMapConfig(randomSeed, widthInTiles: 100, heightInTiles: 100);
                //scenario = new SimulationScenario(
                //   hasFinishedSim: sim => sim.ExplorationTracker.ExploredProportion > 0.99f,
                //   seed: randomSeed,
                //   mapSpawner: generator => generator.GenerateMap(buildingConfig),
                //   robotConstraints: constraints,
                //   robotSpawner: (map, robotSpawner) => robotSpawner.SpawnRobotsTogether(
                //       map,
                //       randomSeed,
                //       1,
                //       new Vector2Int(0, 0),
                //       (seed) => new MinotaurAlgorithm(constraints, randomSeed, 4)
                //   ));

                //            simulator.EnqueueScenario(scenario);
            }
            simulator.PressPlayButton(); // Instantly enter play mode

            //simulator.GetSimulationManager().AttemptSetPlayState(SimulationPlayState.FastAsPossible);
        }
    }
}
