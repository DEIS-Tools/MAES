using System.Collections.Generic;
using Maes;
using Maes.Robot;
using NUnit.Framework;
using UnityEngine;

namespace PlayModeTests {
    public class CommunicationTest {

        private const int MapWidth = 50, MapHeight = 50;
        private const int RandomSeed = 123;
        private Simulator _maes;
        private Simulation _simulation;
        private List<TestingAlgorithm> _robotTestAlgorithms;

        private int[,] GenerateMapWithHorizontalWallInMiddle(int wallThicknessInTiles) {
            int[,] bitmap = new int[MapWidth, MapHeight];
            int firstWallRowY = MapHeight / 2;
            int lastWallRowY = firstWallRowY + wallThicknessInTiles;
            
            for (int x = 0; x < MapWidth; x++) {
                for (int y = 0; y < MapHeight; y++) {
                    var isSolid = y >= firstWallRowY && y <= lastWallRowY - 1;
                    bitmap[x, y] = isSolid ? 1 : 0;
                }
            }

            return bitmap;
        }
        
        
        [TearDown]
        public void ClearSimulator() {
            Simulator.Destroy();
        }

        private void InitSimulator(MapFactory mapFactory, 
            RobotConstraints.SignalTransmissionSuccessCalculator transmissionSuccessCalculatorFunc,
            List<Vector2Int> robotSpawnPositions) {
            var testingScenario = new SimulationScenario(RandomSeed,
                mapSpawner: mapFactory,
                hasFinishedSim: simulation => false,
                robotConstraints: new RobotConstraints(),
                robotSpawner: (map, spawner) => spawner.SpawnRobotsTogether( map, RandomSeed, 2, 
                    Vector2Int.zero, (robotSeed) => {
                        var algorithm = new TestingAlgorithm();
                        _robotTestAlgorithms.Add(algorithm);
                        return algorithm;
                    }));
            
            _maes = Simulator.GetInstance();
            _maes.EnqueueScenario(testingScenario);
            _simulation = _maes.GetSimulationManager().CurrentSimulation;
        }

        
        [Test]
        public void Broadcast_GlobalCommunicationThroughWallsTest() {
            InitSimulator(StandardTestingConfiguration.EmptyCaveMapSpawner(RandomSeed), 
                (distance, wallDistance) => true,
                new List<Vector2Int>());
            
            
        }
        
        [Test]
        public void Broadcast_NoWallsCommunicationTest() {
            //InitSimulator(StandardTestingConfiguration.EmptyCaveMapSpawner(RandomSeed), new List<Vector2Int>());
            
            
        }

        [Test]
        public void Broadcast_1WallTileDetectedTest() {
            InitSimulator((generator => generator.CreateMapFromBitMap(GenerateMapWithHorizontalWallInMiddle(1))),
                (distance, wallDistance) => {
                    if (wallDistance > 0.1f) return false; // Always blocked by walls
                    return true; // Always communicate when in line of sight
                }, 
                new List<Vector2Int>());
        }

    }
}