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

namespace Maes
{
    internal class ExampleProgram : MonoBehaviour
    {
        private Simulator _simulator;
        private void Start()
        {
            const int randomSeed = 269952;
            const int height = 100;
            const int width = 100;

            var bConfig = new BuildingMapConfig(
                randomSeed,
                1,
                width,
                height);

            var constraints = new RobotConstraints(
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

            // Get/instantiate simulation prefab
            var simulator = Simulator.GetInstance();
            for (var leftForce = 0; leftForce < 10; leftForce++)
            {
                for (var rightForce = -10f; rightForce < 0; rightForce++)
                {
                    if (Mathf.Abs(rightForce) == leftForce)
                    {
                        continue;
                    }
                    var buildingConfig = new BuildingMapConfig(
                        randomSeed,
                        3,
                        100,
                        100);
                    var algorithm = new MovementTestAlgorithm(new Vector2Int(50, 47));

                    var scenarioBuilding = new SimulationScenario(
                        seed: randomSeed,
                        mapSpawner: generator => generator.GenerateMap(buildingConfig),
                        robotConstraints: robotConstraints,
                        robotSpawner: (map, robotSpawner) => robotSpawner.SpawnRobotsAtPositions(
                            new List<Vector2Int> { new Vector2Int(0, 0)},
                            map,
                            randomSeed,
                            1,
                            (seed) => algorithm
                        ));
                    //var scenarioBitMap = new SimulationScenario(123, mapSpawner: generator => generator.GenerateMap(bitmap));
                    //simulator.EnqueueScenario(scenarioCave);
                    simulator.EnqueueScenario(scenarioBuilding);
                    //simulator.EnqueueScenario(scenarioBitMap);
                }
            }
            _simulator = simulator;
            EditorApplication.Beep();
            StartCoroutine(PauseSimulation());
            simulator.PressPlayButton(); // Instantly enter play mode
        }

        IEnumerator PauseSimulation()
        {
            while (_simulator.GetSimulationManager().GetCurrentSimulation().SimulatedLogicTicks < 35700)
            {
                yield return new WaitForEndOfFrame();
            }
            _simulator.GetSimulationManager().AttemptSetPlayState(SimulationPlayState.Paused);
            EditorApplication.Beep();
            yield return null;
        }
    }
}