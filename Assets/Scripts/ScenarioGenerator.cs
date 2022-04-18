using System;
using System.Collections.Generic;
using Maes.ExplorationAlgorithm;
using Maes.ExplorationAlgorithm.BrickAndMortar;
using Maes.ExplorationAlgorithm.RandomBallisticWalk;
using Maes.ExplorationAlgorithm.SSB;
using Maes.ExplorationAlgorithm.TheNextFrontier;
using Maes.ExplorationAlgorithm.Voronoi;
using Maes.Map.MapGen;
using Maes.Robot;
using Maes.YamlConfig;
using UnityEngine;
using static Maes.Map.RobotSpawner;

namespace Maes {
    public class ScenarioGenerator {
         private const int Minute = 60;
         
         public static Queue<SimulationScenario> GenerateROS2Scenario() {
             Queue<SimulationScenario> scenarios = new Queue<SimulationScenario>();
             var yamlConfig = MaesYamlConfigLoader.LoadConfig();
             
             // Number of robots
             var numberOfRobots = yamlConfig.NumberOfRobots;
             
             // End criteria
             SimulationEndCriteriaDelegate shouldEndSim = yamlConfig.EndCriteria.CoveragePercent == null
                 ? (simulation) => (simulation.ExplorationTracker
                     .ExploredProportion > yamlConfig.EndCriteria.ExplorationPercent)
                 : (simulation) => (simulation.ExplorationTracker
                     .CoverageProportion > yamlConfig.EndCriteria.CoveragePercent);

             var constraints = new RobotConstraints(
                 broadcastRange: yamlConfig.RobotConstraints.BroadcastRange,
                 broadcastBlockedByWalls: yamlConfig.RobotConstraints.BroadcastBlockedByWalls,
                 senseNearbyAgentsRange: yamlConfig.RobotConstraints.SenseNearbyAgentsRange,
                 senseNearbyAgentsBlockedByWalls: yamlConfig.RobotConstraints.SenseNearbyAgentsBlockedByWalls,
                 automaticallyUpdateSlam: yamlConfig.RobotConstraints.AutomaticallyUpdateSlam,
                 slamUpdateIntervalInTicks: yamlConfig.RobotConstraints.SlamUpdateIntervalInTicks,
                 slamSynchronizeIntervalInTicks: yamlConfig.RobotConstraints.SlamSyncIntervalInTicks,
                 slamPositionInaccuracy: yamlConfig.RobotConstraints.SlamPositionInaccuracy, 
                 distributeSlam: yamlConfig.RobotConstraints.DistributeSlam,
                 environmentTagReadRange: yamlConfig.RobotConstraints.EnvironmentTagReadRange,
                 slamRayTraceRange: yamlConfig.RobotConstraints.SlamRaytraceRange,
                 relativeMoveSpeed: yamlConfig.RobotConstraints.RelativeMoveSpeed,
                 agentRelativeSize: yamlConfig.RobotConstraints.AgentRelativeSize
             );

             foreach (var seed in yamlConfig.RandomSeeds) {
                 MapFactory mapSpawner = null;
                 if (yamlConfig.Map.CaveConfig != null) {
                     var caveConfig = new CaveMapConfig(yamlConfig, seed);
                     mapSpawner = (mapGenerator) => mapGenerator.GenerateCaveMap(caveConfig, yamlConfig.Map.WallHeight);
                 }
                 // Building type
                 else {
                     var buildingConfig = new BuildingMapConfig(yamlConfig, seed);
                     mapSpawner = (mapGenerator) => mapGenerator.GenerateBuildingMap(buildingConfig, yamlConfig.Map.WallHeight);
                 }


                 RobotFactory robotSpawner = null;
                 if (yamlConfig.RobotSpawnConfig.BiggestRoom != null) {
                     robotSpawner = (map, robotSpawner) => robotSpawner.SpawnRobotsInBiggestRoom(
                         collisionMap: map,
                         seed: seed,
                         numberOfRobots: numberOfRobots,
                         (seed) => new Ros2Algorithm());
                 } else if (yamlConfig.RobotSpawnConfig.SpawnTogether != null) {
                     Vector2Int? suggestedStartingPoint = yamlConfig.RobotSpawnConfig.SpawnTogether.HasSuggestedStartingPoint 
                         ? yamlConfig.RobotSpawnConfig.SpawnTogether.SuggestedStartingPointAsVector 
                         : null;
                     robotSpawner = (map, robotSpawner) => robotSpawner.SpawnRobotsTogether(
                         collisionMap: map,
                         seed: seed,
                         numberOfRobots: numberOfRobots,
                         suggestedStartingPoint: suggestedStartingPoint,
                         createAlgorithmDelegate: (seed) => new Ros2Algorithm()
                     );
                 }
                 else { // Spawn_at_hallway_ends
                     robotSpawner = (map, robotSpawner) => robotSpawner.SpawnAtHallWayEnds(
                         collisionMap: map,
                         seed: seed,
                         numberOfRobots: numberOfRobots,
                         createAlgorithmDelegate: (seed) => new Ros2Algorithm()
                     );
                 }

                 scenarios.Enqueue(new SimulationScenario(
                     seed: 0,
                     hasFinishedSim: shouldEndSim,
                     mapSpawner: mapSpawner,
                     robotSpawner: robotSpawner,
                     robotConstraints: constraints,
                     $"{yamlConfig.GlobalSettings.StatisticsResultPath}-{DateTime.Now.Millisecond}"
                 ));
             }

             return scenarios;
         }

         /// <summary>
         /// Generates the scenarios used for the testing of LVD's long-range experiements.
         /// </summary>
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
                senseNearbyAgentsRange: 20f,
                senseNearbyAgentsBlockedByWalls: true,
                automaticallyUpdateSlam: true,
                slamUpdateIntervalInTicks: 10,
                slamSynchronizeIntervalInTicks: 10,
                slamPositionInaccuracy: 0.2f, 
                distributeSlam: false,
                environmentTagReadRange: 0f,
                slamRayTraceRange: 20f,
                relativeMoveSpeed: 1f,
                agentRelativeSize: 0.6f
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
                        1);
                    var buildingConfig = new BuildingMapConfig(
                        width,
                        height,
                        randomSeed,
                        20,
                        4,
                        6,
                        2,
                        2,
                        85,
                        1);
                    
                    foreach (var (algorithmName, createAlgorithmDelegate, constraints) in algorithmsAndFileNames) {
                        scenarios.Enqueue(new SimulationScenario(
                            seed: randomSeed,
                            hasFinishedSim: shouldEndSim,
                            mapSpawner: (mapGenerator) => mapGenerator.GenerateBuildingMap(buildingConfig, 2.0f),
                            robotSpawner: (map, robotSpawner) => robotSpawner.SpawnAtHallWayEnds(
                                map, 
                                randomSeed, 
                                numberOfRobots,
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
                                new Vector2Int(0,0),
                                createAlgorithmDelegate),
                            robotConstraints: constraints,
                            $"{algorithmName}-cave-{width}x{height}-spawnTogether-" + randomSeed
                        ));
                    }
                }
            }

            return scenarios;
         }
         
         /// <summary>
         /// Generates the scenarios used for the YouTube video recordings.
         /// </summary>
        public static Queue<SimulationScenario> GenerateYoutubeVideoScenarios() {
            Queue<SimulationScenario> scenarios = new Queue<SimulationScenario>();
            var numberOfRobots = 2;
            var maxRunTime = 60 * Minute;
            var width = 50;
            var height = 50;
            SimulationEndCriteriaDelegate hasFinishedFunc =
                (simulation) => (simulation.SimulateTimeSeconds >= maxRunTime || simulation.ExplorationTracker.ExploredProportion > 0.995f);

            var robotConstraints = new RobotConstraints(
                broadcastRange: float.MaxValue,
                broadcastBlockedByWalls: false,
                senseNearbyAgentsRange: 7f,
                senseNearbyAgentsBlockedByWalls: true,
                automaticallyUpdateSlam: true,
                slamUpdateIntervalInTicks: 10,
                slamSynchronizeIntervalInTicks: 10,
                slamPositionInaccuracy: 0.2f, 
                distributeSlam: true,
                environmentTagReadRange: 4.0f,
                slamRayTraceRange: 7.0f,
                relativeMoveSpeed: 10f,
                agentRelativeSize: 0.6f
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
                    1);
                var buildingConfig = new BuildingMapConfig(
                    width,
                    height,
                    randomSeed,
                    20,
                    4,
                    6,
                    2,
                    2,
                    85,
                    1);
                
                var algorithmsAndFileNames = new List<(CreateAlgorithmDelegate, string)>()
                {
                    ((seed) => new SsbAlgorithm(robotConstraints, seed),"SSB"),
                    ((seed) => new RandomExplorationAlgorithm(seed), "RBW"),
                    ((seed) => new VoronoiExplorationAlgorithm(seed, robotConstraints, 1), "LVD"),
                };

                foreach (var (createAlgorithmDelegate, algorithmName) in algorithmsAndFileNames) {
                    scenarios.Enqueue(new SimulationScenario(
                        seed: randomSeed,
                        hasFinishedSim: hasFinishedFunc,
                        mapSpawner: (mapGenerator) => mapGenerator.GenerateBuildingMap(buildingConfig, 2.0f),
                        robotSpawner: (map, robotSpawner) => robotSpawner.SpawnAtHallWayEnds(
                            map, 
                            randomSeed, 
                            numberOfRobots,
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
                            new Vector2Int(0,0),
                            createAlgorithmDelegate),
                        robotConstraints: robotConstraints,
                        $"{algorithmName}-cave-{width}x{height}-spawnTogether-" + randomSeed
                    ));
                }
            }
            
            
            

            return scenarios;
        }
        
        /// <summary>
        /// Generates the scenarios used for the main experiments of the MAES paper.
        /// </summary>
        public static Queue<SimulationScenario> GenerateArticleScenarios() {
            Queue<SimulationScenario> scenarios = new Queue<SimulationScenario>();
            var numberOfRobots = 1;
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
                senseNearbyAgentsRange: 7f,
                senseNearbyAgentsBlockedByWalls: true,
                automaticallyUpdateSlam: true,
                slamUpdateIntervalInTicks: 10,
                slamSynchronizeIntervalInTicks: 10,
                slamPositionInaccuracy: 0.2f, 
                distributeSlam: false,
                environmentTagReadRange: 0f,
                slamRayTraceRange: 7f,
                relativeMoveSpeed: 1f,
                agentRelativeSize: 0.6f
            );
            
            var robotConstraintsTNF = new RobotConstraints(
                broadcastRange: 15,
                broadcastBlockedByWalls: true,
                senseNearbyAgentsRange: 12f,
                senseNearbyAgentsBlockedByWalls: true,
                automaticallyUpdateSlam: true,
                slamUpdateIntervalInTicks: 10,
                slamSynchronizeIntervalInTicks: 10,
                slamPositionInaccuracy: 0.2f, 
                distributeSlam: false,
                environmentTagReadRange: 0f,
                slamRayTraceRange: 7f,
                relativeMoveSpeed: 1f,
                agentRelativeSize: 0.6f
            );
            
            var robotConstraintsRBW = new RobotConstraints(
                broadcastRange: 0,
                broadcastBlockedByWalls: false,
                senseNearbyAgentsRange: 0,
                senseNearbyAgentsBlockedByWalls: false,
                automaticallyUpdateSlam: false,
                slamUpdateIntervalInTicks: 10,
                slamSynchronizeIntervalInTicks: 10,
                slamPositionInaccuracy: 0.2f, 
                distributeSlam: false,
                environmentTagReadRange: 0f,
                slamRayTraceRange: 7f,
                relativeMoveSpeed: 1f,
                agentRelativeSize: 0.6f
            );
            
            var robotConstraintsSSB = new RobotConstraints(
                broadcastRange: float.MaxValue,
                broadcastBlockedByWalls: false,
                senseNearbyAgentsRange: 7.0f,
                senseNearbyAgentsBlockedByWalls: false,
                automaticallyUpdateSlam: true,
                slamUpdateIntervalInTicks: 10,
                slamSynchronizeIntervalInTicks: 10,
                slamPositionInaccuracy: 0.2f, 
                distributeSlam: true,
                environmentTagReadRange: 0f,
                slamRayTraceRange: 7f,
                relativeMoveSpeed: 1f,
                agentRelativeSize: 0.6f
            ); 

            for (int i = 0; i < runs; i++) { 
                int randomSeed = i;
                var algorithmsAndFileNames = new List<(string, CreateAlgorithmDelegate, RobotConstraints)>()
                {
                    ("LVD", (seed) => new VoronoiExplorationAlgorithm(seed, robotConstraintsLVD, 1), robotConstraintsLVD),
                    ("RBW", (seed) => new RandomExplorationAlgorithm(seed), robotConstraintsRBW),
                    ("SSB", (seed) => new SsbAlgorithm(robotConstraintsSSB, seed), robotConstraintsSSB),
                    ("TNF", (seed) => new TnfExplorationAlgorithm(8, 8, seed), robotConstraintsTNF)
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
                        1);
                    var buildingConfig = new BuildingMapConfig(
                        width,
                        height,
                        randomSeed,
                        20,
                        4,
                        6,
                        2,
                        2,
                        85,
                        1);
                    
                    foreach (var (algorithmName, createAlgorithmDelegate, constraints) in algorithmsAndFileNames) {
                        scenarios.Enqueue(new SimulationScenario(
                            seed: randomSeed,
                            hasFinishedSim: algorithmName == "TNF" ? shouldEndTnfSim : shouldEndSim,
                            mapSpawner: (mapGenerator) => mapGenerator.GenerateBuildingMap(buildingConfig, 2.0f),
                            robotSpawner: (map, robotSpawner) => robotSpawner.SpawnAtHallWayEnds(
                                map, 
                                randomSeed, 
                                numberOfRobots,
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
                                new Vector2Int(0,0),
                                createAlgorithmDelegate),
                            robotConstraints: constraints,
                            $"{algorithmName}-cave-{width}x{height}-spawnTogether-" + randomSeed
                        ));
                    }
                }
            }

            return scenarios;
        }
        
        /// <summary>
        /// Generates scenarios with the LVD algorithm.
        /// </summary>
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
                    1);

                var buildingConfig = new BuildingMapConfig(
                    50,
                    50,
                    randomSeed,
                    20,
                    4,
                    6,
                    2,
                    2,
                    85,
                    1);

                var robotConstraints = new RobotConstraints(
                    broadcastRange: float.MaxValue,
                    broadcastBlockedByWalls: false,
                    senseNearbyAgentsRange: 10f,
                    senseNearbyAgentsBlockedByWalls: true,
                    automaticallyUpdateSlam: true,
                    slamUpdateIntervalInTicks: 10,
                    slamSynchronizeIntervalInTicks: 10,
                    slamPositionInaccuracy: 0.2f, 
                    distributeSlam: true,
                    environmentTagReadRange: 4.0f,
                    slamRayTraceRange: 7f,
                    relativeMoveSpeed: 1f,
                    agentRelativeSize: 0.6f
                );

                if (i % 2 != 0) {
                    scenarios.Enqueue(new SimulationScenario(
                        seed: randomSeed,
                        hasFinishedSim: (simulation) => simulation.SimulateTimeSeconds >= 20 * minute,
                        mapSpawner: (mapGenerator) => mapGenerator.GenerateBuildingMap(buildingConfig, 2.0f),
                        robotSpawner: (map, robotSpawner) => robotSpawner.SpawnAtHallWayEnds(
                            map, 
                            randomSeed, 
                            1,
                            (seed) => new VoronoiExplorationAlgorithm(seed, robotConstraints, 1)),
                        robotConstraints: robotConstraints,
                        "Voronoi-building-hallway-" + randomSeed
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
                            new Vector2Int(0,0),
                            (seed) => new VoronoiExplorationAlgorithm(seed, robotConstraints, 1)),
                        robotConstraints: robotConstraints,
                        "Voronoi-cave-together-" + randomSeed
                    ));
                }
            }

            return scenarios; 
        }
        
        /// <summary>
        /// Generates scenarios with the RBW algorithm.
        /// </summary>
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
                    1);

                var buildingConfig = new BuildingMapConfig(
                    30,
                    30,
                    randomSeed,
                    58,
                    4,
                    5,
                    2,
                    1,
                    75,
                    1);

                
                var robotConstraints = new RobotConstraints(
                    broadcastRange: 15.0f,
                    broadcastBlockedByWalls: true,
                    senseNearbyAgentsRange: 10f,
                    senseNearbyAgentsBlockedByWalls: true,
                    automaticallyUpdateSlam: true,
                    slamUpdateIntervalInTicks: 10,
                    slamSynchronizeIntervalInTicks: 10,
                    slamPositionInaccuracy: 0.2f,
                    distributeSlam: false,
                    environmentTagReadRange: 4.0f,
                    slamRayTraceRange: 7.0f,
                    relativeMoveSpeed: 1f,
                    agentRelativeSize: 0.6f
                );

                if (i % 2 == 0) {
                    scenarios.Enqueue(new SimulationScenario(
                        seed: randomSeed,
                        hasFinishedSim: (simulation) => simulation.SimulateTimeSeconds >= 60 * minute,
                        mapSpawner: (mapGenerator) => mapGenerator.GenerateBuildingMap(buildingConfig, 2.0f),
                        robotSpawner: (map, robotSpawner) => robotSpawner.SpawnAtHallWayEnds(
                            map, 
                            randomSeed, 
                            2,
                            (seed) => new RandomExplorationAlgorithm(seed)),
                        robotConstraints: robotConstraints,
                        "RBW-building-" + randomSeed
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
                            new Vector2Int(0,0),
                            (seed) => new VoronoiExplorationAlgorithm(seed, robotConstraints, 2)),
                        robotConstraints: robotConstraints,
                        "RBW-hallway-" + randomSeed
                    ));
                }
            }

            return scenarios;
        }
        
        /// <summary>
        /// Generates scenarios with the BNM algorithm.<br/>
        /// <b>WARNING!!!</b>: Algorithm not fully implemented. Will not work as advertised.
        /// </summary>
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
                    1);

                var buildingConfig = new BuildingMapConfig(
                    60,
                    60,
                    randomSeed,
                    58,
                    4,
                    5,
                    2,
                    1,
                    75,
                    1);

                var robotConstraints = new RobotConstraints(
                    broadcastRange: 15.0f,
                    broadcastBlockedByWalls: true,
                    senseNearbyAgentsRange: 5f,
                    senseNearbyAgentsBlockedByWalls: true,
                    automaticallyUpdateSlam: true,
                    slamUpdateIntervalInTicks: 10,
                    slamSynchronizeIntervalInTicks: 10,
                    slamPositionInaccuracy: 0.5f,
                    distributeSlam: false,
                    environmentTagReadRange: 4.0f,
                    slamRayTraceRange: 7.0f,
                    relativeMoveSpeed: 1f,
                    agentRelativeSize: 0.6f
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
                    mapSpawner: (mapGenerator) => mapGenerator.GenerateBuildingMap(buildingConfig, 2.0f),
                    robotSpawner: (map, robotSpawner) => robotSpawner.SpawnAtHallWayEnds(
                        map, 
                        randomSeed, 
                        1,
                        (seed) => new BrickAndMortar(robotConstraints, seed)),
                    robotConstraints: robotConstraints,
                    "BM-building-hallway-" + randomSeed
                ));
            }

            return scenarios;
        }
     
        /// <summary>
        /// Generates scenarios with the SSB algorithm.
        /// </summary>
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
                    1);

                var buildingConfig = new BuildingMapConfig(
                    60,
                    60,
                    randomSeed,
                    58,
                    4,
                    5,
                    2,
                    1,
                    75,
                    1);

                var robotConstraints = new RobotConstraints(
                    broadcastRange: float.MaxValue,
                    broadcastBlockedByWalls: false,
                    senseNearbyAgentsRange: 5f,
                    senseNearbyAgentsBlockedByWalls: true,
                    automaticallyUpdateSlam: true,
                    slamUpdateIntervalInTicks: 10,
                    slamSynchronizeIntervalInTicks: 10,
                    slamPositionInaccuracy: 0.2f,
                    distributeSlam: true,
                    environmentTagReadRange: 4.0f,
                    slamRayTraceRange: 7.0f,
                    relativeMoveSpeed: 1f,
                    agentRelativeSize: 0.6f
                );

                scenarios.Enqueue(new SimulationScenario(
                    seed: randomSeed,
                    hasFinishedSim: (simulation) => simulation.SimulateTimeSeconds >= 60 * minute,
                    mapSpawner: (mapGenerator) => mapGenerator.GenerateCaveMap(caveConfig, 2.0f),
                    robotSpawner: (map, robotSpawner) => robotSpawner.SpawnRobotsInBiggestRoom(
                        map, 
                        randomSeed, 
                        5,
                        (seed) => new SsbAlgorithm(robotConstraints, seed)),
                    robotConstraints: robotConstraints,
                    "SSB-cave-biggestroom-" + randomSeed
                ));
            }

            return scenarios;
        }

        /// <summary>
        /// Generates scenarios with the TNF algorithm.
        /// </summary>
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
                1);

            var buildingConfig = new BuildingMapConfig(
                200,
                200,
                randomSeed,
                58,
                4,    
                5,
                2,
                1,
                75,
                1);

            var robotConstraints = new RobotConstraints(
                broadcastRange: 15.0f,
                broadcastBlockedByWalls: true,
                senseNearbyAgentsRange: 5f,
                senseNearbyAgentsBlockedByWalls: true,
                automaticallyUpdateSlam: true,
                slamUpdateIntervalInTicks: 10,
                slamSynchronizeIntervalInTicks: 10,
                slamPositionInaccuracy: 0.2f,
                distributeSlam: false,
                environmentTagReadRange: 4.0f,
                slamRayTraceRange: 7f,
                relativeMoveSpeed: 1f,
                agentRelativeSize: 0.6f
            );
            
            scenarios.Enqueue(new SimulationScenario(
                seed: randomSeed, 
                hasFinishedSim: simulation => simulation.SimulateTimeSeconds >= 60 * Minute,
                mapSpawner: generator => generator.GenerateBuildingMap(config: buildingConfig, 2.0f),
                robotSpawner: (map, robotSpawner) => robotSpawner.SpawnAtHallWayEnds(
                    map,
                    randomSeed,
                    15,
                    (seed) => new TnfExplorationAlgorithm(5, 9, randomSeed)),
                robotConstraints: robotConstraints,
                "TNF-building-test-" + randomSeed
            ));

            return scenarios;
        }
    }
}