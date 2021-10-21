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

                var officeConfig = new OfficeMapConfig(
                    60,
                    60,
                    randomSeed, 
                    58,
                    4,
                    5,
                    3,
                    1,
                    70,
                    2,
                    2f);
                
                scenarios.Enqueue(new SimulationScenario(
                    seed: randomSeed,
                    hasFinishedSim: (simulation) => simulation.SimulateTimeSeconds >= 300,
                    mapSpawner: (mapGenerator) => mapGenerator.GenerateCaveMap(mapConfig, 3.0f, true),
                    robotSpawner: (map, robotSpawner) => robotSpawner.SpawnRobotsTogether(map, randomSeed, 40, new Coord(40,20)),
                    robotConstraints: new RobotConstraints()
                ));
            }

            return scenarios;
        }
    }
}