// Copyright 2022 MAES
// 
// This file is part of MAES
// 
// MAES is free software: you can redistribute it and/or modify it under
// the terms of the GNU General Public License as published by the
// Free Software Foundation, either version 3 of the License, or (at your option)
// any later version.
// 
// MAES is distributed in the hope that it will be useful, but WITHOUT
// ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
// or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
// Public License for more details.
// 
// You should have received a copy of the GNU General Public License along
// with MAES. If not, see http://www.gnu.org/licenses/.
// 
// Contributors: Malte Z. Andreasen, Philip I. Holler and Magnus K. Jensen
// 
// Original repository: https://github.com/MalteZA/MAES

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
    public class CommunicationTest
    {

        private const int MapWidth = 50, MapHeight = 50;
        private const int RandomSeed = 123;
        private Simulator _maes;
        private Simulation _simulation;
        private List<TestingAlgorithm> _robotTestAlgorithms;

        private Tile[,] GenerateMapWithHorizontalWallInMiddle(int wallThicknessInTiles)
        {
            Tile[,] bitmap = new Tile[MapWidth, MapHeight];
            int firstWallRowY = MapHeight / 2;
            int lastWallRowY = firstWallRowY + wallThicknessInTiles;
            Tile.Rand = new Random(RandomSeed);
            var wall = Tile.GetRandomWall();

            for (int x = 0; x < MapWidth; x++)
            {
                for (int y = 0; y < MapHeight; y++)
                {
                    var isSolid = y >= firstWallRowY && y <= lastWallRowY - 1;
                    bitmap[x, y] = isSolid ? wall : new Tile(TileType.Room);
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
            RobotConstraints.SignalTransmissionSuccessCalculator transmissionSuccessCalculatorFunc,
            List<Vector2Int> robotSpawnPositions)
        {
            _robotTestAlgorithms = new List<TestingAlgorithm>();
            var testingScenario = new SimulationScenario(RandomSeed,
                mapSpawner: mapFactory,
                hasFinishedSim: simulation => false,
                robotConstraints: new RobotConstraints(materialCommunication: false, calculateSignalTransmissionProbability: transmissionSuccessCalculatorFunc),
                robotSpawner: (map, spawner) => spawner.SpawnRobotsAtPositions(robotSpawnPositions, map, RandomSeed, 2,
                    robotSeed => {
                        var algorithm = new TestingAlgorithm();
                        _robotTestAlgorithms.Add(algorithm);
                        return algorithm;
                    }));

            _maes = Simulator.GetInstance();
            _maes.EnqueueScenario(testingScenario);
            _simulation = _maes.GetSimulationManager().CurrentSimulation;

            // The first robot will broadcast immediatealy
            _robotTestAlgorithms[0].UpdateFunction = (tick, controller) => {
                if (tick == 0) controller.Broadcast("Test Message");
            };

            // The second robot will continuously receive broadcasts
            _robotTestAlgorithms[0].UpdateFunction = (tick, controller) => {
                controller.ReceiveBroadcast();
            };
        }


        [Test(ExpectedResult = null)]
        public IEnumerator Broadcast_TransmissionSuccessTest()
        {
            InitSimulator(StandardTestingConfiguration.EmptyCaveMapSpawner(RandomSeed),
                (distance, wallDistance) => true,
                new List<Vector2Int> { new Vector2Int(2, 2), new Vector2Int(6, 6) });

            string receivedMessage = null;
            string sentMessage = "message sent between robots 1 and 2";
            var algorithm1 = _robotTestAlgorithms[0];
            var algorithm2 = _robotTestAlgorithms[1];

            algorithm1.UpdateFunction = (tick, controller) => {
                if (tick == 0) controller.Broadcast(sentMessage);
            };

            algorithm2.UpdateFunction = (tick, controller) => {
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
            InitSimulator(StandardTestingConfiguration.EmptyCaveMapSpawner(RandomSeed),
                (distance, wallDistance) => false, // Always fail communication
                new List<Vector2Int> { new Vector2Int(2, 2), new Vector2Int(6, 6) });

            string receivedMessage = null;
            string sentMessage = "message sent between robots 1 and 2";
            var algorithm1 = _robotTestAlgorithms[0];
            var algorithm2 = _robotTestAlgorithms[1];

            algorithm1.UpdateFunction = (tick, controller) => {
                if (tick == 0) controller.Broadcast(sentMessage);
            };

            algorithm2.UpdateFunction = (tick, controller) => {
                var results = controller.ReceiveBroadcast();
                if (results.Count != 0) receivedMessage = results[0] as string;
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
            float foundWallDistance = float.PositiveInfinity;

            InitSimulator(StandardTestingConfiguration.EmptyCaveMapSpawner(RandomSeed),
                (distance, wallDistance) => {
                    foundWallDistance = wallDistance;
                    return true;
                },
                new List<Vector2Int> { new Vector2Int(-10, -10), new Vector2Int(10, 10) });

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
        [TestCase(20, 20, ExpectedResult = null)]
        [TestCase(5, 5, ExpectedResult = null)]
        [TestCase(5, -5, ExpectedResult = null)]
        [TestCase(-5, 5, ExpectedResult = null)]
        [TestCase(-5, -5, ExpectedResult = null)]
        public IEnumerator Broadcast_CorrectDistanceCalculation(int secondRobotX, int secondRobotY)
        {
            float transmissionDistance = float.PositiveInfinity;
            Vector2Int firstRobotPosition = new Vector2Int(0, 0);
            Vector2Int secondRobotPosition = new Vector2Int(secondRobotX, secondRobotY);
            float actualDistance = (firstRobotPosition - secondRobotPosition).magnitude;

            InitSimulator(StandardTestingConfiguration.EmptyCaveMapSpawner(RandomSeed),
                (distance, wallDistance) => {
                    transmissionDistance = distance;
                    return true;
                },
                new List<Vector2Int> { firstRobotPosition, secondRobotPosition });

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
                    transmissionSuccessCalculatorFunc:
                    (distance, wallDistance) => {
                        foundWallDistance = wallDistance;
                        return true;
                    },
                new List<Vector2Int> { new Vector2Int(0, -2), new Vector2Int(0, 3 + wallThickness) });

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