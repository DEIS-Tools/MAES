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
using Maes.ExplorationAlgorithm.RandomBallisticWalk;
using Maes.Map;
using Maes.Map.MapGen;
using Maes.Utilities.Files;
using UnityEngine;

namespace Maes {
    internal class ExampleProgram : MonoBehaviour {
        private void Start() {
            const int randomSeed = 123;
            const int height = 100;
            const int width = 100;
            var buildingConfig = new BuildingMapConfig(
                randomSeed,
                width,
                height,
                20,
                4,
                6,
                2,
                2,
                85,
                1);
            var caveConfig = new CaveMapConfig(
                randomSeed,
                width,
                height,
                4,
                4,
                45,
                10,
                10,
                1);
            var bitmap = PgmMapFileLoader.LoadMapFromFileIfPresent("map.pgm");

            // Get/instantiate simulation prefab
            var simulator = Simulator.GetInstance();
            
            // Setup configuration for a scenario
            var scenarioCave = new SimulationScenario(123, mapSpawner: generator => generator.GenerateMap(caveConfig));
            var scenarioBuilding = new SimulationScenario(123, mapSpawner: generator => generator.GenerateMap(buildingConfig));
            var scenarioBitMap = new SimulationScenario(123, mapSpawner: generator => generator.GenerateMap(bitmap));
            simulator.EnqueueScenario(scenarioCave);
            simulator.EnqueueScenario(scenarioBuilding);
            simulator.EnqueueScenario(scenarioBitMap);
        
            simulator.PressPlayButton(); // Instantly enter play mode
        }
    }
}