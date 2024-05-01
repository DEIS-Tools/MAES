using System.Collections;
using System.Collections.Generic;
using Maes;
using Maes.Map.MapGen;
using Maes.Robot;
using NUnit.Framework;
using UnityEngine;
using Random = System.Random;


namespace PlayModeTests
{
    public class MaterialCommunicationTest
    {
        private const int MapWidth = 50, MapHeight = 50;
        private const int RandomSeed = 123;
        private Simulator _maes;
        private Simulation _simulation;
        private List<TestingAlgorithm> _robotTestAlgorithms;

        private static Tile[,] GenerateMapWithHorizontalWallInMiddle(int wallThicknessInTiles)
        {
            var bitmap = new Tile[MapWidth, MapHeight];
            const int firstWallRowY = MapHeight / 2;
            var lastWallRowY = firstWallRowY + wallThicknessInTiles;
            Tile.Rand = new Random(RandomSeed);
            var wall = Tile.GetRandomWall();

            for (var x = 0; x < MapWidth; x++)
            {
                for (var y = 0; y < MapHeight; y++)
                {
                    var isSolid = y >= firstWallRowY && y <= lastWallRowY - 1;
                    bitmap[x, y] = isSolid ? wall : new Tile(TileType.Room);
                }
            }

            return bitmap;
        }
        private static Tile[,] GenerateMapWithHorizontalAlternatingWallInMiddle()
        {
            const int wallThicknessInTiles = 3;
            var bitmap = new Tile[MapWidth, MapHeight];
            const int firstWallRowY = MapHeight / 2;
            const int lastWallRowY = firstWallRowY + wallThicknessInTiles;
            var walls = new List<Tile>()
            {
                new(TileType.Brick),
                new(TileType.Wood),
                new(TileType.Concrete)
            };

            for (var x = 0; x < MapWidth; x++)
            {
                for (var y = 0; y < MapHeight; y++)
                {
                    var isSolid = y is >= firstWallRowY and <= lastWallRowY - 1;
                    bitmap[x, y] = isSolid ? walls[y - firstWallRowY] : new Tile(TileType.Room);
                }
            }

            return bitmap;
        }


        [TearDown]
        public void ClearSimulator()
        {
            Simulator.Destroy();
        }

        private void InitSimulator(MapFactory mapFactory,
            List<Vector2Int> robotSpawnPositions,
                Dictionary<uint, Dictionary<TileType, float>> attenuationDictionary = null,
            RobotConstraints.SignalTransmissionSuccessCalculator transmissionSuccessCalculatorFunc = null
            )
        {
            _robotTestAlgorithms = new List<TestingAlgorithm>();
            var testingScenario = new SimulationScenario(RandomSeed,
                mapSpawner: mapFactory,
                hasFinishedSim: _ => false,
                robotConstraints: new RobotConstraints(materialCommunication: true, calculateSignalTransmissionProbability: transmissionSuccessCalculatorFunc, attenuationDictionary: attenuationDictionary),
                robotSpawner: (map, spawner) => spawner.SpawnRobotsAtPositions(robotSpawnPositions, map, RandomSeed, 2,
                    _ =>
                    {
                        var algorithm = new TestingAlgorithm();
                        _robotTestAlgorithms.Add(algorithm);
                        return algorithm;
                    }));

            _maes = Simulator.GetInstance();
            _maes.EnqueueScenario(testingScenario);
            _simulation = _maes.GetSimulationManager().CurrentSimulation;

            // The first robot will broadcast immediately
            _robotTestAlgorithms[0].UpdateFunction = (tick, controller) =>
            {
                if (tick == 0) controller.Broadcast("Test Message");
            };

            // The second robot will continuously receive broadcasts
            _robotTestAlgorithms[0].UpdateFunction = (_, controller) =>
            {
                controller.ReceiveBroadcast();
            };
        }


        [Test(ExpectedResult = null)]
        public IEnumerator Broadcast_TransmissionSuccessTest()
        {
            InitSimulator(StandardTestingConfiguration.EmptyCaveMapSpawner(RandomSeed),
                new List<Vector2Int> { new(2, 2), new(6, 6) });

            string receivedMessage = null;
            const string sentMessage = "message sent between robots 1 and 2";
            var algorithm1 = _robotTestAlgorithms[0];
            var algorithm2 = _robotTestAlgorithms[1];

            algorithm1.UpdateFunction = (tick, controller) =>
            {
                if (tick == 0) controller.Broadcast(sentMessage);
            };

            algorithm2.UpdateFunction = (_, controller) =>
            {
                var results = controller.ReceiveBroadcast();
                if (results.Count != 0) receivedMessage = results[0] as string;
            };

            _maes.PressPlayButton();
            // Wait until the message has been broadcast
            while (_simulation.SimulatedLogicTicks < 2)
            {
                yield return null;
            }

            Assert.AreEqual(sentMessage, receivedMessage);
        }

        [Test(ExpectedResult = null)]
        public IEnumerator Broadcast_TransmissionFailedTest()
        {
            InitSimulator(generator => generator.GenerateMap(GenerateMapWithHorizontalWallInMiddle(1), RandomSeed, borderSize: 2),
                new List<Vector2Int> { new(-4, -4), new(8, 8) },
                new Dictionary<uint, Dictionary<TileType, float>>
                {
                    [2400] = new() //2.4 GHz
                    {
                        [TileType.Room] = 0f,
                        [TileType.Hall] = 0f,
                        [TileType.Wall] = 0f,
                        [TileType.Concrete] = 100f,
                        [TileType.Wood] = 100f,
                        [TileType.Brick] = 100f
                    }
                });

            string receivedMessage = null;
            const string sentMessage = "message sent between robots 1 and 2";
            var algorithm1 = _robotTestAlgorithms[0];
            var algorithm2 = _robotTestAlgorithms[1];

            algorithm1.UpdateFunction = (tick, controller) =>
            {
                if (tick == 0)
                    controller.Broadcast(sentMessage);
            };

            algorithm2.UpdateFunction = (_, controller) =>
            {
                var results = controller.ReceiveBroadcast();
                if (results.Count != 0)
                    receivedMessage = results[0] as string;
            };

            _maes.PressPlayButton();
            // Wait until the message has been broadcast
            while (_simulation.SimulatedLogicTicks < 2)
            {
                yield return null;
            }

            Assert.IsNull(receivedMessage);
        }

        [Test(ExpectedResult = null)]
        public IEnumerator Broadcast_NoWallsCommunicationTest()
        {
            var foundWallDistance = float.PositiveInfinity;

            InitSimulator(StandardTestingConfiguration.EmptyCaveMapSpawner(RandomSeed),
                new List<Vector2Int> { new(-10, -10), new(10, 10) },
                transmissionSuccessCalculatorFunc: (_, wallDistance) =>
                {
                    foundWallDistance = wallDistance;
                    return true;
                });

            _maes.PressPlayButton();
            // Wait until the message has been broadcast
            while (_simulation.SimulatedLogicTicks < 2)
            {
                yield return null;
            }

            // Assert that the signal is said to travel through 1 meter/unit of wall
            Assert.AreEqual(foundWallDistance, 0f, 0.001f);
        }


        [Test(ExpectedResult = null)]
        public IEnumerator AttenuationCalculation()
        {
            InitSimulator(generator => generator.GenerateMap(GenerateMapWithHorizontalAlternatingWallInMiddle(), RandomSeed, borderSize: 2),
                new List<Vector2Int> { new(-4, -4), new(8, 8) },
                new Dictionary<uint, Dictionary<TileType, float>>
                {
                    [2400] = new() //2.4 GHz
                    {
                        [TileType.Room] = 0f,
                        [TileType.Hall] = 0f,
                        [TileType.Wall] = 0f,
                        [TileType.Concrete] = 1f,
                        [TileType.Wood] = 2f,
                        [TileType.Brick] = 3f
                    }
                });

            _maes.PressPlayButton();
            // Wait until the adjacency matrix has been created
            while (_simulation._communicationManager.CommunicationTracker.AdjacencyMatrixRef == null)
            {
                yield return null;
            }

            var signalStrength = _simulation._communicationManager.CommunicationTracker.AdjacencyMatrixRef[(0,1)].SignalStrength;

            Assert.AreEqual(-27, signalStrength);
        }

        [Test(ExpectedResult = null)]
        [TestCase(20, 20, ExpectedResult = null)]
        [TestCase(5, 5, ExpectedResult = null)]
        [TestCase(5, -5, ExpectedResult = null)]
        [TestCase(-5, 5, ExpectedResult = null)]
        [TestCase(-5, -5, ExpectedResult = null)]
        public IEnumerator Broadcast_CorrectDistanceCalculation(int secondRobotX, int secondRobotY)
        {
            var transmissionDistance = float.PositiveInfinity;
            var firstRobotPosition = new Vector2Int(0, 0);
            var secondRobotPosition = new Vector2Int(secondRobotX, secondRobotY);
            var actualDistance = (firstRobotPosition - secondRobotPosition).magnitude;

            InitSimulator(StandardTestingConfiguration.EmptyCaveMapSpawner(RandomSeed),
                new List<Vector2Int> { firstRobotPosition, secondRobotPosition },
                transmissionSuccessCalculatorFunc: (distance, _) =>
                {
                    transmissionDistance = distance;
                    return true;
                });

            _maes.PressPlayButton();
            // Wait until the message has been broadcast
            while (_simulation.SimulatedLogicTicks < 2)
            {
                yield return null;
            }

            // Assert that the signal is said to travel through 1 meter/unit of wall
            Assert.AreEqual(actualDistance, transmissionDistance, 0.001f);
        }

        [Test(ExpectedResult = null)]
        [TestCase(1, ExpectedResult = null)]
        [TestCase(2, ExpectedResult = null)]
        [TestCase(5, ExpectedResult = null)]
        [TestCase(10, ExpectedResult = null)]
        public IEnumerator Broadcast_WallDistanceIsApproximatelyCorrect(int wallThickness)
        {
            float foundWallDistance = float.PositiveInfinity;
            InitSimulator(
                generator => generator.GenerateMap(GenerateMapWithHorizontalWallInMiddle(wallThickness), RandomSeed, borderSize: 2),
                new List<Vector2Int> { new(0, -2), new(0, 3 + wallThickness) },
                transmissionSuccessCalculatorFunc:
                (_, wallDistance) =>
                {
                    foundWallDistance = wallDistance;
                    return true;
                });

            _maes.PressPlayButton();
            // Wait until the message has been broadcast
            while (_simulation.SimulatedLogicTicks < 5)
            {
                yield return null;
            }

            // Assert that the signal is said to travel through 1 meter/unit of wall
            Assert.AreEqual((float)wallThickness, foundWallDistance, 0.1f * wallThickness);
        }
    }
}
