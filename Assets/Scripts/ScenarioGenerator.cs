using System;
using System.Collections.Generic;
using Dora.ExplorationAlgorithm;
using Dora.ExplorationAlgorithm.SSB;
using Dora.ExplorationAlgorithm.Voronoi;
using Dora.ExplorationAlgorithm.TheNextFrontier;
using Dora.MapGeneration;
using static Dora.MapGeneration.RobotSpawner;

namespace Dora {
    public class ScenarioGenerator {
         private const int Minute = 60;
         
         public static Queue<SimulationScenario> GenerateVoronoiLongRangeScenarios() {
             Queue<SimulationScenario> scenarios = new Queue<SimulationScenario>();
            var numberOfRobots = 15;
            var runs = 20;
            var sizes = new List<(int, int)>() {(200,200)};
            var maxRunTime = 60 * Minute;
            SimulationEndCriteriaDelegate shouldEndSim = (simulation) => (simulation.SimulateTimeSeconds >= maxRunTime
                                                                             || simulation.ExplorationTracker
                                                                                 .CoverageProportion > 0.995f);
            
            var robotConstraintsLVD = new RobotConstraints(
                broadcastRange: 0,
                broadcastBlockedByWalls: false,
                senseNearbyRobotRange: 20f,
                senseNearbyRobotBlockedByWalls: true,
                automaticallyUpdateSlam: true,
                slamUpdateIntervalInTicks: 10,
                slamSynchronizeIntervalInTicks: 10,
                slamPositionInaccuracy: 0.2f, 
                distributeSlam: false,
                environmentTagReadRange: 0f,
                lidarRange: 20f
            );
            
            for (int i = 0; i < runs; i++) { 
                int randomSeed = i;
                var algorithmsAndFileNames = new List<(string, CreateAlgorithmDelegate, RobotConstraints)>()
                {
                    ("LVD-long-range", (seed) => new VoronoiExplorationAlgorithm(seed, robotConstraintsLVD, 1), robotConstraintsLVD),
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
            var sizes = new List<(int, int)>() {(50, 50), (100,100), (200,200)};
            var maxRunTime = 60 * Minute;
            SimulationEndCriteriaDelegate shouldEndSim = (simulation) => (simulation.SimulateTimeSeconds >= maxRunTime
                                                                             || simulation.ExplorationTracker
                                                                                 .CoverageProportion > 0.995f);
            // Overwrite for when simulating TNF, which aims at exploration, and not coverage.
            SimulationEndCriteriaDelegate shouldEndTnfSim = simulation => simulation.SimulateTimeSeconds >= maxRunTime
                                                                          || simulation.ExplorationTracker
                                                                              .ExploredProportion > .995f
                                                                          || simulation.TnfBotsOutOfFrontiers();
            
            
            var robotConstraintsLVD = new RobotConstraints(
                broadcastRange: 0,
                broadcastBlockedByWalls: false,
                senseNearbyRobotRange: 7f,
                senseNearbyRobotBlockedByWalls: true,
                automaticallyUpdateSlam: true,
                slamUpdateIntervalInTicks: 10,
                slamSynchronizeIntervalInTicks: 10,
                slamPositionInaccuracy: 0.2f, 
                distributeSlam: false,
                environmentTagReadRange: 0f,
                lidarRange: 7f
            );
            
            var robotConstraintsTNF = new RobotConstraints(
                broadcastRange: 15,
                broadcastBlockedByWalls: true,
                senseNearbyRobotRange: 12f,
                senseNearbyRobotBlockedByWalls: true,
                automaticallyUpdateSlam: true,
                slamUpdateIntervalInTicks: 10,
                slamSynchronizeIntervalInTicks: 10,
                slamPositionInaccuracy: 0.2f, 
                distributeSlam: false,
                environmentTagReadRange: 0f,
                lidarRange: 7f
            );
            
            var robotConstraintsRBW = new RobotConstraints(
                broadcastRange: 0,
                broadcastBlockedByWalls: false,
                senseNearbyRobotRange: 0,
                senseNearbyRobotBlockedByWalls: false,
                automaticallyUpdateSlam: false,
                slamUpdateIntervalInTicks: 10,
                slamSynchronizeIntervalInTicks: 10,
                slamPositionInaccuracy: 0.2f, 
                distributeSlam: false,
                environmentTagReadRange: 0f,
                lidarRange: 0f
            );
            
            var robotConstraintsSSB = new RobotConstraints(
                broadcastRange: float.MaxValue,
                broadcastBlockedByWalls: false,
                senseNearbyRobotRange: 7.0f,
                senseNearbyRobotBlockedByWalls: false,
                automaticallyUpdateSlam: true,
                slamUpdateIntervalInTicks: 10,
                slamSynchronizeIntervalInTicks: 10,
                slamPositionInaccuracy: 0.2f, 
                distributeSlam: true,
                environmentTagReadRange: 0f,
                lidarRange: 7f
            ); 

            for (int i = 0; i < runs; i++) { 
                int randomSeed = i;
                var algorithmsAndFileNames = new List<(string, CreateAlgorithmDelegate, RobotConstraints)>()
                {
                    ("TNF", (seed) => new TnfExplorationAlgorithm(8, 8, seed), robotConstraintsTNF),
                    ("SSB", (seed) => new SsbAlgorithm(robotConstraintsSSB, seed), robotConstraintsSSB),
                    ("LVD", (seed) => new VoronoiExplorationAlgorithm(seed, robotConstraintsLVD, 1), robotConstraintsLVD),
                    ("RBW", (seed) => new RandomExplorationAlgorithm(seed), robotConstraintsRBW),
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
                            hasFinishedSim: algorithmName == "TNF" ? shouldEndTnfSim : shouldEndSim,
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
                            hasFinishedSim: algorithmName == "TNF" ? shouldEndTnfSim : shouldEndSim,
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

        public static Queue<SimulationScenario> GenerateTnfScenarios() {
            var scenarios = new Queue<SimulationScenario>();

            int randomSeed = 4 + 2;

            var mapConfig = new CaveMapConfig(
                100,
                100,
                randomSeed,
                4,
                2,
                48,
                10,
                1,
                1,
                1f);

            var officeConfig = new OfficeMapConfig(
                200,
                200,
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
                slamPositionInaccuracy: 0.2f,
                distributeSlam: false,
                environmentTagReadRange: 4.0f,
                lidarRange: 7f
            );
            
            scenarios.Enqueue(new SimulationScenario(
                seed: randomSeed, 
                hasFinishedSim: simulation => simulation.SimulateTimeSeconds >= 60 * Minute,
                mapSpawner: generator => generator.GenerateOfficeMap(config: officeConfig, 2.0f),
                robotSpawner: (map, robotSpawner) => robotSpawner.SpawnAtHallWayEnds(
                    map,
                    randomSeed,
                    15,
                    0.6f,
                    (seed) => new TnfExplorationAlgorithm(5, 9, randomSeed)),
                robotConstraints: robotConstraints,
                "TNF-office-test-" + randomSeed
            ));

            return scenarios;
        }
    }
}