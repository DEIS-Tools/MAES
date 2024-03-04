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
using Maes.Map;
using Maes.Map.MapGen;
using Maes.Robot;
using Maes.Utilities.Files;
using UnityEngine;
using Maes.Robot;
using ExplorationAlgorithm;
using System.Collections.Generic;

namespace Maes
{
    internal class ExampleProgram : MonoBehaviour
    {
        private void Start()
        {
            const int randomSeed = 123;
            var bitmap = PgmMapFileLoader.LoadMapFromFileIfPresent("Blank_map.pgm");

            var robotConstraints = new RobotConstraints(
                senseNearbyAgentsRange: 5f,
                senseNearbyAgentsBlockedByWalls: true,
                automaticallyUpdateSlam: true,
                slamUpdateIntervalInTicks: 10,
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


            var buildingConfig = new BuildingMapConfig(randomSeed: randomSeed, widthInTiles: 100, heightInTiles: 100);

            // Get/instantiate simulation prefab
            var simulator = Simulator.GetInstance();
            var rightForce = 1f;
            for (var leftForce = -0.625f; leftForce < 0.625f; leftForce += 0.001f)
            {
                if (Mathf.Abs(rightForce) == Mathf.Abs(leftForce))
                {
                    continue;
                }
                var scenarioBitMap = new SimulationScenario(
                seed: randomSeed,
                mapSpawner: (gen) => gen.GenerateMap(bitmap, randomSeed),
                robotSpawner: (map, spawner) => spawner.SpawnRobotsAtPositions(
                    new List<Vector2Int> { new Vector2Int(0, 0) },
                    map,
                    randomSeed,
                    1,
                    seed => new MinotaurAlgorithm(robotConstraints, seed)
                ),
                robotConstraints: robotConstraints
            );

                simulator.EnqueueScenario(scenarioBitMap);
            }
            simulator.PressPlayButton(); // Instantly enter play mode
        }
    }
}