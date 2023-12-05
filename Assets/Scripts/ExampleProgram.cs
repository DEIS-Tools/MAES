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
using System.Collections;
using System.Collections.Generic;
using Maes.ExplorationAlgorithm.TheNextFrontier;
using Maes.Map;
using Maes.Map.MapGen;
using Maes.Robot;
using Maes.Utilities.Files;
using UnityEngine;

namespace Maes
{
    internal class ExampleProgram : MonoBehaviour
    {
        private void Start()
        {
            const int randomSeed = 123;
            const int height = 50;
            const int width = 50;
            var caveConfig = new CaveMapConfig(
                randomSeed,
                width,
                height);
            var bitmap = PgmMapFileLoader.LoadMapFromFileIfPresent("map.pgm");

            // Get/instantiate simulation prefab
            var simulator = Simulator.GetInstance();

            // Setup configuration for a scenario
            var scenarioCave = new SimulationScenario(
                hasFinishedSim: (sim) => sim.ExplorationTracker.ExploredProportion > 0.5f,
                seed: randomSeed,
                mapSpawner: generator => generator.GenerateMap(caveConfig),
                robotSpawner: (map, robotSpawner) => robotSpawner.SpawnRobotsTogether(
                    map,
                    randomSeed,
                    5,
                    new Vector2Int(0, 0),
                    (seed) => new TnfExplorationAlgorithm(1, 10, seed)
                ));
            for (var i = 0; i < 10; i++)
            {
                var buildingConfig = new BuildingMapConfig(
                    randomSeed+i,
                    3,
                    100,
                    100);

                var scenarioBuilding = new SimulationScenario(
                    hasFinishedSim: sim => sim.ExplorationTracker.ExploredProportion > 0.3f || sim.SimulatedLogicTicks > 360,
                    seed: randomSeed+i,
                    mapSpawner: generator => generator.GenerateMap(buildingConfig),
                    robotSpawner: (map, robotSpawner) => robotSpawner.SpawnRobotsTogether(
                        map,
                        randomSeed+i,
                        5,
                        new Vector2Int(0, 0),
                        (seed) => new TnfExplorationAlgorithm(1, 10, seed)
                    ));
                //var scenarioBitMap = new SimulationScenario(123, mapSpawner: generator => generator.GenerateMap(bitmap));
                //simulator.EnqueueScenario(scenarioCave);
                simulator.EnqueueScenario(scenarioBuilding);
                //simulator.EnqueueScenario(scenarioBitMap);
            }

            simulator.PressPlayButton(); // Instantly enter play mode
        }
    }
}