using System.Collections.Generic;
using Dora.ExplorationAlgorithm;
using Dora.ExplorationAlgorithm.SSB;
using Dora.ExplorationAlgorithm.Voronoi;
using Dora.MapGeneration;

namespace Dora {
    public class ScenarioGenerator {

        public static Queue<SimulationScenario> GenerateVoronoiScenarios() {
           Queue<SimulationScenario> scenarios = new Queue<SimulationScenario>();

            for (int i = 0; i < 1; i++) {
                int randomSeed = i + 4 + 1;
                int minute = 60;
                var mapConfig = new CaveMapConfig(
                    50,
                    50,
                    randomSeed,
                    4,
                    4,
                    45,
                    10,
                    1,
                    1,
                    1f);

                var officeConfig = new OfficeMapConfig(
                    60,
                    60,
                    randomSeed,
                    20,
                    4,
                    4,
                    2,
                    1,
                    85,
                    1,
                    1f);

                
                var robotConstraints = new RobotConstraints(
                    broadcastRange: 15.0f,
                    broadcastBlockedByWalls: true,
                    senseNearbyRobotRange: 10f,
                    senseNearbyRobotBlockedByWalls: true,
                    automaticallyUpdateSlam: true,
                    slamUpdateIntervalInTicks: 10,
                    slamSynchronizeIntervalInTicks: 10,
                    slamPositionInaccuracy: 0.2f,
                    distributeSlam: false,
                    environmentTagReadRange: 4.0f
                );

                if (i % 2 != 0) {
                    scenarios.Enqueue(new SimulationScenario(
                        seed: randomSeed,
                        hasFinishedSim: (simulation) => simulation.SimulateTimeSeconds >= 20 * minute,
                        mapSpawner: (mapGenerator) => mapGenerator.GenerateOfficeMap(officeConfig, 2.0f),
                        robotSpawner: (map, robotSpawner) => robotSpawner.SpawnAtHallWayEnds(
                            map, 
                            randomSeed, 
                            3, 
                            0.6f,
                            (seed) => new VoronoiExplorationAlgorithm(seed, robotConstraints, 2)),
                        robotConstraints: robotConstraints,
                        "Voronoi-office-hallway-" + randomSeed
                    ));
                }
                else {
                    scenarios.Enqueue(new SimulationScenario(
                        seed: randomSeed,
                        hasFinishedSim: (simulation) => simulation.SimulateTimeSeconds >= 10 * minute,
                        mapSpawner: (mapGenerator) => mapGenerator.GenerateCaveMap(mapConfig, 2.0f),
                        robotSpawner: (map, robotSpawner) => robotSpawner.SpawnRobotsTogether(
                            map, 
                            randomSeed, 
                            1, 
                            0.6f,
                            new Coord(20,20),
                            (seed) => new VoronoiExplorationAlgorithm(seed, robotConstraints, 1)),
                        robotConstraints: robotConstraints,
                        "Voronoi-cave-together-" + randomSeed
                    ));
                }
            }

            return scenarios; 
        }
        
        public static Queue<SimulationScenario> GenerateBallisticScenarios() {
            Queue<SimulationScenario> scenarios = new Queue<SimulationScenario>();

            for (int i = 0; i < 5; i++) {
                int randomSeed = i + 4 + 1;
                int minute = 60;
                var mapConfig = new CaveMapConfig(
                    60,
                    60,
                    randomSeed,
                    4,
                    2,
                    0,
                    10,
                    1,
                    1,
                    1f);

                var officeConfig = new OfficeMapConfig(
                    30,
                    30,
                    randomSeed,
                    58,
                    4,
                    5,
                    2,
                    1,
                    75,
                    1,
                    1f);

                
                var robotConstraints = new RobotConstraints(
                    broadcastRange: 15.0f,
                    broadcastBlockedByWalls: true,
                    senseNearbyRobotRange: 10f,
                    senseNearbyRobotBlockedByWalls: true,
                    automaticallyUpdateSlam: true,
                    slamUpdateIntervalInTicks: 10,
                    slamSynchronizeIntervalInTicks: 10,
                    slamPositionInaccuracy: 0.2f,
                    distributeSlam: false,
                    environmentTagReadRange: 4.0f
                );

                if (i % 2 == 0) {
                    scenarios.Enqueue(new SimulationScenario(
                        seed: randomSeed,
                        hasFinishedSim: (simulation) => simulation.SimulateTimeSeconds >= 20 * minute,
                        mapSpawner: (mapGenerator) => mapGenerator.GenerateOfficeMap(officeConfig, 2.0f),
                        robotSpawner: (map, robotSpawner) => robotSpawner.SpawnAtHallWayEnds(
                            map, 
                            randomSeed, 
                            2, 
                            0.6f,
                            (seed) => new RandomExplorationAlgorithm(seed)),
                        robotConstraints: robotConstraints,
                        "RBW-office-" + randomSeed
                    ));
                }
                else {
                    scenarios.Enqueue(new SimulationScenario(
                        seed: randomSeed,
                        hasFinishedSim: (simulation) => simulation.SimulateTimeSeconds >= 20 * minute,
                        mapSpawner: (mapGenerator) => mapGenerator.GenerateCaveMap(mapConfig, 2.0f),
                        robotSpawner: (map, robotSpawner) => robotSpawner.SpawnRobotsTogether(
                            map, 
                            randomSeed, 
                            1, 
                            0.6f,
                            new Coord(0,0),
                            (seed) => new VoronoiExplorationAlgorithm(seed, robotConstraints, 2)),
                        robotConstraints: robotConstraints,
                        "RBW-cave-" + randomSeed
                    ));
                }
            }

            return scenarios;
        }
        
        public static Queue<SimulationScenario> GenerateBrickAndMortarScenarios() {
            Queue<SimulationScenario> scenarios = new Queue<SimulationScenario>();

            for (int i = 0; i < 5; i++) {
                int randomSeed = i + 4 + 1;
                int minute = 60;
                var mapConfig = new CaveMapConfig(
                    60,
                    60,
                    randomSeed,
                    4,
                    2,
                    48,
                    10,
                    1,
                    1,
                    1f);

                var officeConfig = new OfficeMapConfig(
                    60,
                    60,
                    randomSeed,
                    58,
                    4,
                    5,
                    2,
                    1,
                    75,
                    1,
                    1f);

                var robotConstraints = new RobotConstraints(
                    broadcastRange: 15.0f,
                    broadcastBlockedByWalls: true,
                    senseNearbyRobotRange: 5f,
                    senseNearbyRobotBlockedByWalls: true,
                    automaticallyUpdateSlam: true,
                    slamUpdateIntervalInTicks: 10,
                    slamSynchronizeIntervalInTicks: 10,
                    slamPositionInaccuracy: 0.5f,
                    distributeSlam: false,
                    environmentTagReadRange: 4.0f
                );
                
                /*scenarios.Enqueue(new SimulationScenario(
                    seed: randomSeed,
                    hasFinishedSim: (simulation) => simulation.SimulateTimeSeconds >= 60 * minute,
                    mapSpawner: (mapGenerator) => mapGenerator.GenerateCaveMap(mapConfig, 2.0f),
                    robotSpawner: (map, robotSpawner) => robotSpawner.SpawnRobotsInBiggestRoom(
                        map, 
                        randomSeed, 
                        1, 
                        0.6f,
                        (seed) => new BrickAndMortar(robotConstraints, seed)),
                    robotConstraints: robotConstraints
                ));*/
                
                scenarios.Enqueue(new SimulationScenario(
                    seed: randomSeed,
                    hasFinishedSim: (simulation) => simulation.SimulateTimeSeconds >= 60 * minute,
                    mapSpawner: (mapGenerator) => mapGenerator.GenerateOfficeMap(officeConfig, 2.0f),
                    robotSpawner: (map, robotSpawner) => robotSpawner.SpawnAtHallWayEnds(
                        map, 
                        randomSeed, 
                        1, 
                        0.6f,
                        (seed) => new BrickAndMortar(robotConstraints, seed)),
                    robotConstraints: robotConstraints,
                    "BNM-office-" + randomSeed
                ));
            }

            return scenarios;
        }
     
         public static Queue<SimulationScenario> GenerateSsbScenarios() {
            Queue<SimulationScenario> scenarios = new Queue<SimulationScenario>();

            for (int i = 0; i < 5; i++) {
                int randomSeed = i + 4 + 1;
                int minute = 60;
                var mapConfig = new CaveMapConfig(
                    60,
                    60,
                    randomSeed,
                    4,
                    2,
                    48,
                    10,
                    1,
                    1,
                    1f);

                var officeConfig = new OfficeMapConfig(
                    60,
                    60,
                    randomSeed,
                    58,
                    4,
                    5,
                    2,
                    1,
                    75,
                    1,
                    1f);

                var robotConstraints = new RobotConstraints(
                    broadcastRange: 15.0f,
                    broadcastBlockedByWalls: true,
                    senseNearbyRobotRange: 5f,
                    senseNearbyRobotBlockedByWalls: true,
                    automaticallyUpdateSlam: true,
                    slamUpdateIntervalInTicks: 10,
                    slamSynchronizeIntervalInTicks: 10,
                    slamPositionInaccuracy: 0.0f, // TODO!
                    distributeSlam: false,
                    environmentTagReadRange: 4.0f
                );

                scenarios.Enqueue(new SimulationScenario(
                    seed: randomSeed,
                    hasFinishedSim: (simulation) => simulation.SimulateTimeSeconds >= 60 * minute,
                    mapSpawner: (mapGenerator) => mapGenerator.GenerateOfficeMap(officeConfig, 2.0f),
                    robotSpawner: (map, robotSpawner) => robotSpawner.SpawnAtHallWayEnds(
                        map, 
                        randomSeed, 
                        1, 
                        0.6f,
                        (seed) => new SsbAlgorithm(robotConstraints, seed)),
                    robotConstraints: robotConstraints,
                    "SSB-office-" + randomSeed
                ));
            }

            return scenarios;
        }
    }
}