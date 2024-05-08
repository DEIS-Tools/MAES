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

namespace Maes
{
    internal class GreedExploratoryExperiments : MonoBehaviour
    {
        private Simulator _simulator;
        private void Start()
        {
            const int randomSeed = 12345;

            var LOS = new RobotConstraints(
                senseNearbyAgentsRange: 5f,
                senseNearbyAgentsBlockedByWalls: true,
                automaticallyUpdateSlam: true,
                slamUpdateIntervalInTicks: 1,
                slamSynchronizeIntervalInTicks: 10,
                slamPositionInaccuracy: 0.2f,
                distributeSlam: false,
                environmentTagReadRange: 4.0f,
                slamRayTraceRange: 4f,
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

            var map = PgmMapFileLoader.LoadMapFromFileIfPresent("blank_100.pgm");
            var random = new System.Random(1234);
            for (int i = 0; i < 10; i++)
            {
                var val = random.Next(0, 1000000);
                simulator.EnqueueScenario(new SimulationScenario(val,
                    mapSpawner: generator => generator.GenerateMap(map, val),
                    robotConstraints: LOS,
                    statisticsFileName: $"mino_blank_{val}",
                    robotSpawner: (map, spawner) => spawner.SpawnRobotsTogether(map,
                        val,
                        5,
                        new Vector2Int(0, 0),
                        robotSeed => new MinotaurAlgorithm(LOS, val, 4))));
                simulator.EnqueueScenario(new SimulationScenario(val,
                    mapSpawner: generator => generator.GenerateMap(map, val),
                    robotConstraints: LOS,
                    statisticsFileName: $"greed_blank_{val}",
                    robotSpawner: (map, spawner) => spawner.SpawnRobotsTogether(map,
                        val,
                        5,
                        new Vector2Int(0, 0),
                        robotSeed => new GreedAlgorithm())));
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
                                                                 createAlgorithmDelegate: (seed) => new MinotaurAlgorithm(LOS, seed, 4)),
                statisticsFileName: $"delete-me",
                robotConstraints: LOS));

            simulator.PressPlayButton(); // Instantly enter play mode

            //simulator.GetSimulationManager().AttemptSetPlayState(SimulationPlayState.FastAsPossible);
        }
    }
}
