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
            const int randomSeed = 1; // 948778

            var constraints = new RobotConstraints(
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
                    // Max distance 15.0f
                    else if (15.0f < distanceTravelled)
                    {
                        return false;
                    }

                    return true;
                }
            );

            var map = PgmMapFileLoader.LoadMapFromFileIfPresent("doorway.pgm");

            var scenario = new SimulationScenario(
                seed: randomSeed,
                mapSpawner: generator => generator.GenerateMap(map, randomSeed),
                robotConstraints: constraints,
                robotSpawner: (map, robotSpawner) => robotSpawner.SpawnRobotsAtPositions(
                    new List<Vector2Int> { new Vector2Int(0, 0) },
                    map,
                    randomSeed,
                    1,
                    (seed) => new MinotaurAlgorithm(constraints, randomSeed, 4)
                ));
            // Get/instantiate simulation prefab
            var simulator = Simulator.GetInstance();

            var buildingConfig = new CaveMapConfig(randomSeed, widthInTiles: 100, heightInTiles: 100);
            scenario = new SimulationScenario(
               hasFinishedSim: sim => sim.ExplorationTracker.ExploredProportion > 0.99f,
               seed: randomSeed,
               mapSpawner: generator => generator.GenerateMap(buildingConfig),
               robotConstraints: constraints,
               robotSpawner: (map, robotSpawner) => robotSpawner.SpawnRobotsTogether(
                   map,
                   randomSeed,
                   4,
                   new Vector2Int(0, 0),
                   (seed) => new MinotaurAlgorithm(constraints, randomSeed, 4)
               ));

            simulator.EnqueueScenario(scenario);
            simulator.PressPlayButton(); // Instantly enter play mode

            //simulator.GetSimulationManager().AttemptSetPlayState(SimulationPlayState.FastAsPossible);
        }
    }
}