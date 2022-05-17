using System;
using Maes.Map.MapGen;
using UnityEngine;

namespace Maes {
    internal class ExampleProgram : MonoBehaviour {
        private void Start() {
            // Get/instantiate simulation prefab
            var simulator = Simulator.GetInstance();
            
            
            
            // Setup configuration for a scenario
            //var caveConfig = new CaveMapConfig(123, widthInTiles: 75, heightInTiles: 75);
            //var scenario = new SimulationScenario(123, mapSpawner: generator => generator.GenerateCaveMap(caveConfig));
            //simulator.EnqueueScenario(scenario);
            
            //simulator.StartSimulation();
            simulator.DefaultStart(true);
        }
    }
}