using System;
using System.Collections.Generic;
using Dora.ExplorationAlgorithm;
using Dora.ExplorationAlgorithm.SSB;
using Dora.ExplorationAlgorithm.Voronoi;
using Dora.MapGeneration;
using static Dora.MapGeneration.RobotSpawner;

namespace Dora {
    public class ScenarioGenerator {
         private const int Minute = 60;

        public static Queue<SimulationScenario> GenerateYoutubeVideoScenarios() {
            Queue<SimulationScenario> scenarios = new Queue<SimulationScenario>();
            var numberOfRobots = 1;
            var maxRunTime = 60 * Minute;
            var width = 50;
            var height = 50;
            SimulationEndCriteriaDelegate hasFinishedFunc =
                (simulation) => (simulation.SimulateTimeSeconds >= maxRunTime);

            var robotConstraints = new RobotConstraints(
                broadcastRange: float.MaxValue,
                broadcastBlockedByWalls: false,
                senseNearbyRobotRange: 7f,
                senseNearbyRobotBlockedByWalls: true,
                automaticallyUpdateSlam: true,
                slamUpdateIntervalInTicks: 10,
                slamSynchronizeIntervalInTicks: 10,
                slamPositionInaccuracy: 0.2f, 
                distributeSlam: true,
                environmentTagReadRange: 4.0f,
                lidarRange: 7.0f
            );

            for (int i = 0; i < 1; i++) {
                var randomSeed = i;
                
                var caveConfig = new CaveMapConfig(
                    width,
                    height,
                    randomSeed,
                    4,
                    4,
                    45,
                    10,
                    10,
                    1,
                    1f);
                var officeConfig = new OfficeMapConfig(
                    width,
                    height,
                    randomSeed,
                    20,
                    4,
                    6,
                    2,
                    2,
                    85,
                    1,
                    1f);
                
                var algorithmsAndFileNames = new List<(CreateAlgorithmDelegate, string)>()
                {
                    ((seed) => new SsbAlgorithm(robotConstraints, seed),"SSB"),
                    ((seed) => new VoronoiExplorationAlgorithm(seed, robotConstraints, 1), "LVD"),
                    ((seed) => new RandomExplorationAlgorithm(seed), "RBW"),
                    // TODO: Include The next frontier
                };

                foreach (var (createAlgorithmDelegate, algorithmName) in algorithmsAndFileNames) {
                    scenarios.Enqueue(new SimulationScenario(
                        seed: randomSeed,
                        hasFinishedSim: hasFinishedFunc,
                        mapSpawner: (mapGenerator) => mapGenerator.GenerateOfficeMap(officeConfig, 2.0f),
                        robotSpawner: (map, robotSpawner) => robotSpawner.SpawnAtHallWayEnds(
                            map, 
                            randomSeed, 
                            numberOfRobots, 
                            0.6f,
                            createAlgorithmDelegate),
                        robotConstraints: robotConstraints,
                        $"{algorithmName}-building-{width}x{height}-hallway-" + randomSeed
                    ));
                    scenarios.Enqueue(new SimulationScenario(
                        seed: randomSeed,
                        hasFinishedSim: hasFinishedFunc,
                        mapSpawner: (mapGenerator) => mapGenerator.GenerateCaveMap(caveConfig, 2.0f),
                        robotSpawner: (map, robotSpawner) => robotSpawner.SpawnRobotsTogether(
                            map, 
                            randomSeed, 
                            numberOfRobots, 
                            0.6f,
                            new Coord(0,0),
                            createAlgorithmDelegate),
                        robotConstraints: robotConstraints,
                        $"{algorithmName}-cave-{width}x{height}-spawnTogether-" + randomSeed
                    ));
                }
            }
            
            
            

            return scenarios;
        }
        
        
        public static Queue<SimulationScenario> GenerateArticleScenarios() {
            Queue<SimulationScenario> scenarios = new Queue<SimulationScenario>();
            var numberOfRobots = 15;
            var runs = 20;
            var sizes = new List<(int, int)>() {(50, 50), (100, 100), (200,200)};
            var maxRunTime = 60 * Minute;
            SimulationEndCriteriaDelegate shouldEndSim = (simulation) => (simulation.SimulateTimeSeconds >= maxRunTime
                                                                             || simulation.ExplorationTracker
                                                                                 .CoverageProportion > 0.995f); 
            var robotConstraintsBlockedByWalls = new RobotConstraints(
                broadcastRange: float.MaxValue,
                broadcastBlockedByWalls: true,
                senseNearbyRobotRange: 7f,
                senseNearbyRobotBlockedByWalls: true,
                automaticallyUpdateSlam: true,
                slamUpdateIntervalInTicks: 10,
                slamSynchronizeIntervalInTicks: 10,
                slamPositionInaccuracy: 0.2f, 
                distributeSlam: true,
                environmentTagReadRange: 4.0f,
                lidarRange: 7f
            );
            
            // This will short circuit the population of the adjacency, which 
            // can improve the simulation performance of algorithms, where communication
            // is not blocked by walls significantly, e.g. SSB.
            // This does not effect the performance in terms of exploration
            // since if all signals can travels through walls, we don't care about ray tracing and counting number
            // of walls encountered.
            var robotConstraintsThroughWalls = new RobotConstraints(
                broadcastRange: float.MaxValue,
                broadcastBlockedByWalls: false,
                senseNearbyRobotRange: 7f,
                senseNearbyRobotBlockedByWalls: false,
                automaticallyUpdateSlam: true,
                slamUpdateIntervalInTicks: 10,
                slamSynchronizeIntervalInTicks: 10,
                slamPositionInaccuracy: 0.2f, 
                distributeSlam: true,
                environmentTagReadRange: 4.0f,
                lidarRange: 7f
            );
            
            
            for (int i = 0; i < runs; i++) { 
                int randomSeed = i;
                var algorithmsAndFileNames = new List<(string, CreateAlgorithmDelegate, RobotConstraints)>()
                {
                    ("SSB", (seed) => new SsbAlgorithm(robotConstraintsThroughWalls, seed), robotConstraintsThroughWalls),
                    //("LVD", (seed) => new VoronoiExplorationAlgorithm(seed, robotConstraintsBlockedByWalls, 1), robotConstraintsBlockedByWalls),
                    //("RBW", (seed) => new RandomExplorationAlgorithm(seed), robotConstraintsThroughWalls),
                    //("BNM", (seed) => new BrickAndMortar(robotConstraintsThroughWalls, seed), robotConstraintsThroughWalls),
                };
                foreach (var (width, height) in sizes) {
                    var caveConfig = new CaveMapConfig(
                        width,
                        height,
                        randomSeed,
                        4,
                        4,
                        45,
                        10,
                        10,
                        1,
                        1f);
                    var officeConfig = new OfficeMapConfig(
                        width,
                        height,
                        randomSeed,
                        20,
                        4,
                        6,
                        2,
                        2,
                        85,
                        1,
                        1f);
                    
                    foreach (var (algorithmName, createAlgorithmDelegate, constraints) in algorithmsAndFileNames) {
                        scenarios.Enqueue(new SimulationScenario(
                            seed: randomSeed,
                            hasFinishedSim: shouldEndSim,
                            mapSpawner: (mapGenerator) => mapGenerator.GenerateOfficeMap(officeConfig, 2.0f),
                            robotSpawner: (map, robotSpawner) => robotSpawner.SpawnAtHallWayEnds(
                                map, 
                                randomSeed, 
                                numberOfRobots, 
                                0.6f,
                                createAlgorithmDelegate),
                            robotConstraints: constraints,
                            $"{algorithmName}-building-{width}x{height}-hallway-" + randomSeed
                        ));
                        scenarios.Enqueue(new SimulationScenario(
                            seed: randomSeed,
                            hasFinishedSim: shouldEndSim,
                            mapSpawner: (mapGenerator) => mapGenerator.GenerateCaveMap(caveConfig, 2.0f),
                            robotSpawner: (map, robotSpawner) => robotSpawner.SpawnRobotsTogether(
                                map, 
                                randomSeed, 
                                numberOfRobots, 
                                0.6f,
                                new Coord(0,0),
                                createAlgorithmDelegate),
                            robotConstraints: constraints,
                            $"{algorithmName}-cave-{width}x{height}-spawnTogether-" + randomSeed
                        ));
                    }
                }
            }

            return scenarios;
        }
        
        public static Queue<SimulationScenario> GenerateVoronoiScenarios() {
           Queue<SimulationScenario> scenarios = new Queue<SimulationScenario>();

            for (int i = 0; i < 3; i++) {
                int randomSeed = i + 4 + 1;
                int minute = 60;
                var mapConfig = new CaveMapConfig(
                    10,
                    10,
                    randomSeed,
                    4,
                    4,
                    0,
                    10,
                    1,
                    1,
                    1f);

                var officeConfig = new OfficeMapConfig(
                    50,
                    50,
                    randomSeed,
                    20,
                    4,
                    6,
                    2,
                    2,
                    85,
                    1,
                    1f);

                var robotConstraints = new RobotConstraints(
                    broadcastRange: float.MaxValue,
                    broadcastBlockedByWalls: false,
                    senseNearbyRobotRange: 10f,
                    senseNearbyRobotBlockedByWalls: true,
                    automaticallyUpdateSlam: true,
                    slamUpdateIntervalInTicks: 10,
                    slamSynchronizeIntervalInTicks: 10,
                    slamPositionInaccuracy: 0.2f, 
                    distributeSlam: true,
                    environmentTagReadRange: 4.0f,
                    lidarRange: 7f
                );

                if (i % 2 != 0) {
                    scenarios.Enqueue(new SimulationScenario(
                        seed: randomSeed,
                        hasFinishedSim: (simulation) => simulation.SimulateTimeSeconds >= 20 * minute,
                        mapSpawner: (mapGenerator) => mapGenerator.GenerateOfficeMap(officeConfig, 2.0f),
                        robotSpawner: (map, robotSpawner) => robotSpawner.SpawnAtHallWayEnds(
                            map, 
                            randomSeed, 
                            1, 
                            0.6f,
                            (seed) => new VoronoiExplorationAlgorithm(seed, robotConstraints, 1)),
                        robotConstraints: robotConstraints,
                        "Voronoi-office-hallway-" + randomSeed
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
                    environmentTagReadRange: 4.0f,
                    lidarRange: 7.0f
                );

                if (i % 2 == 0) {
                    scenarios.Enqueue(new SimulationScenario(
                        seed: randomSeed,
                        hasFinishedSim: (simulation) => simulation.SimulateTimeSeconds >= 60 * minute,
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
                        "RBW-hallway-" + randomSeed
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
                    environmentTagReadRange: 4.0f,
                    lidarRange: 7.0f
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
                    "BM-office-hallway-" + randomSeed
                ));
            }

            return scenarios;
        }
     
         public static Queue<SimulationScenario> GenerateSsbScenarios() {
            Queue<SimulationScenario> scenarios = new Queue<SimulationScenario>();

            for (int i = 0; i < 5; i++) {
                int randomSeed = i + 4 + 1;
                int minute = 60;
                var caveConfig = new CaveMapConfig(
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
                    broadcastRange: float.MaxValue,
                    broadcastBlockedByWalls: false,
                    senseNearbyRobotRange: 5f,
                    senseNearbyRobotBlockedByWalls: true,
                    automaticallyUpdateSlam: true,
                    slamUpdateIntervalInTicks: 10,
                    slamSynchronizeIntervalInTicks: 10,
                    slamPositionInaccuracy: 0.2f,
                    distributeSlam: true,
                    environmentTagReadRange: 4.0f,
                    lidarRange: 7.0f
                );

                // scenarios.Enqueue(new SimulationScenario(
                //     seed: randomSeed,
                //     hasFinishedSim: (simulation) => simulation.SimulateTimeSeconds >= 60 * minute,
                //     mapSpawner: (mapGenerator) => mapGenerator.GenerateOfficeMap(officeConfig, 2.0f),
                //     robotSpawner: (map, robotSpawner) => robotSpawner.SpawnAtHallWayEnds(
                //         map, 
                //         randomSeed, 
                //         5, 
                //         0.6f,
                //         (seed) => new SsbAlgorithm(robotConstraints, seed)),
                //     robotConstraints: robotConstraints
                // ));
                
                scenarios.Enqueue(new SimulationScenario(
                    seed: randomSeed,
                    hasFinishedSim: (simulation) => simulation.SimulateTimeSeconds >= 60 * minute,
                    mapSpawner: (mapGenerator) => mapGenerator.GenerateCaveMap(caveConfig, 2.0f),
                    robotSpawner: (map, robotSpawner) => robotSpawner.SpawnRobotsInBiggestRoom(
                        map, 
                        randomSeed, 
                        5, 
                        0.6f,
                        (seed) => new SsbAlgorithm(robotConstraints, seed)),
                    robotConstraints: robotConstraints,
                    "SSB-cave-biggestroom-" + randomSeed
                ));
            }

            return scenarios;
        }
    }
}