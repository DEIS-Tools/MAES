using System.Collections.Generic;
using Dora.MapGeneration;

namespace Dora
{
    public class ScenarioGenerator
    {
        public static Queue<SimulationScenario> GenerateBallisticScenarios()
        {
            Queue<SimulationScenario> scenarios = new Queue<SimulationScenario>();
            
            for (int i = 0; i < 5; i++)
            {
                int randomSeed = i + 4;
                var mapConfig = new CaveMapConfig(60,
                    60,
                    randomSeed,
                    4,
                    2,
                    48,
                    10,
                    1,
                    1,
                    2);
                
                scenarios.Enqueue(new SimulationScenario(
                    seed: randomSeed,
                    hasFinishedSim: (simulation) => simulation.SimulateTimeSeconds >= 60,
                    mapSpawner: (mapGenerator) => mapGenerator.GenerateCaveMap(mapConfig, 3.0f, true),
                    robotSpawner: (map, robotSpawner) => robotSpawner.SpawnRobots(map, randomSeed),
                    new RobotConstraints()
                ));
            }

            return scenarios;
        }
    }
}