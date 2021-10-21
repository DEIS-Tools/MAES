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
                var mapConfig = new CaveMapConfig(
                    140,
                    140,
                    randomSeed,
                    4,
                    2,
                    48,
                    10,
                    1,
                    1,
                    2);

                var officeConfig = new OfficeMapConfig(
                    80,
                    80,
                    randomSeed, 
                    58,
                    4,
                    5,
                    3,
                    1,
                    75,
                    2,
                    2f);
                
                var robotConstraints = new RobotConstraints(
                    broadcastRange: 15.0f,
                    broadcastBlockedByWalls: true
                );
                
                scenarios.Enqueue(new SimulationScenario(
                    seed: randomSeed,
                    hasFinishedSim: (simulation) => simulation.SimulateTimeSeconds >= 300,
                    mapSpawner: (mapGenerator) => mapGenerator.GenerateOfficeMap(officeConfig, 3.0f, true),
                    robotSpawner: (map, robotSpawner) => robotSpawner.SpawnRobotsTogether(map, randomSeed, 40),
                    robotConstraints
                ));
            }

            return scenarios;
        }
    }
}