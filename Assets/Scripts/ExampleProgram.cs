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
using UnityEngine;

namespace Maes {
    internal class ExampleProgram : MonoBehaviour {
        private void Start() {
            // Get/instantiate simulation prefab
            var simulator = Simulator.GetInstance();
            
            
            simulator.DefaultStart(false);
            // Setup configuration for a scenario
            //var caveConfig = new CaveMapConfig(123, widthInTiles: 75, heightInTiles: 75);
            //var scenario = new SimulationScenario(123, mapSpawner: generator => generator.GenerateCaveMap(caveConfig));
            //simulator.EnqueueScenario(scenario);
        
            // simulator.StartSimulation(); // Instantly enter play mode
        }
    }
}