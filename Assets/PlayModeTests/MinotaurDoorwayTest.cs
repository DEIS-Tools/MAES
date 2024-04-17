using System.Collections;
using System.Collections.Generic;
using Maes;
using Maes.Robot;
using NUnit.Framework;
using UnityEngine;
using Maes.ExplorationAlgorithm.Minotaur;
using Maes.Utilities.Files;
using System.Linq;

namespace PlayModeTests
{
    public class MinotaurDoorwayMock : MinotaurAlgorithm
    {
        public MinotaurDoorwayMock(RobotConstraints robotConstraints, int seed, int doorWidth) : base(robotConstraints, seed, doorWidth)
        { }

        public List<Doorway> GetDoorways()
        {
            return _doorways;
        }
    }


    public class MinotaurDoorwayTest
    {
        private const int RandomSeed = 123;
        private Simulator _maes;
        private Simulation _simulation;
        private List<MinotaurDoorwayMock> _minotaurs = new();

        [TearDown]
        public void ClearSimulator()
        {
            Simulator.Destroy();
            _minotaurs.Clear();
        }
        private void InitSimulator(string mapName, List<Vector2Int> robotSpawnPositions)
        {
            var constraints = new RobotConstraints(
                senseNearbyAgentsRange: 5f,
                senseNearbyAgentsBlockedByWalls: true,
                automaticallyUpdateSlam: true,
                slamUpdateIntervalInTicks: 1,
                slamSynchronizeIntervalInTicks: 10,
                slamPositionInaccuracy: 0.2f,
                distributeSlam: false,
                environmentTagReadRange: 4.0f,
                slamRayTraceRange: 4f,
                relativeMoveSpeed: 1f,
                agentRelativeSize: 0.6f,
                calculateSignalTransmissionProbability: (distanceTravelled, distanceThroughWalls) =>
                {
                    // Blocked by walls
                    if (distanceThroughWalls > 0)
                    {
                        return false;
                    }
                    // Max distance 15.0f
                    else if (15.0f < distanceTravelled)
                    {
                        return false;
                    }

                    return true;
                }
            );
            var map = PgmMapFileLoader.LoadMapFromFileIfPresent(mapName + ".pgm");
            var testingScenario = new SimulationScenario(RandomSeed,
                mapSpawner: generator => generator.GenerateMap(map, RandomSeed),
                hasFinishedSim: simulation => false,
                robotConstraints: constraints,
                robotSpawner: (map, spawner) => spawner.SpawnRobotsAtPositions(robotSpawnPositions, map, RandomSeed, 1,
                    robotSeed =>
                    {
                        var algorithm = new MinotaurDoorwayMock(constraints, RandomSeed, 4);
                        _minotaurs.Add(algorithm);
                        return algorithm;
                    }));

            _maes = Simulator.GetInstance();
            _maes.EnqueueScenario(testingScenario);
            _simulation = _maes.GetSimulationManager().CurrentSimulation;
        }

        [Test(ExpectedResult = null)]
        public IEnumerator BlankMap()
        {
            InitSimulator("blank", new List<Vector2Int> { new Vector2Int(0, 0) });

            _maes.PressPlayButton();
            _maes.GetSimulationManager().AttemptSetPlayState(Maes.UI.SimulationPlayState.FastAsPossible);

            while (_simulation.ExplorationTracker.ExploredProportion < 0.99f)
            {
                yield return null;
            }

            Assert.AreEqual(0, _minotaurs.First().GetDoorways().Count);
        }

        [Test(ExpectedResult = null)]
        public IEnumerator SingleDoorway()
        {
            InitSimulator("doorway", new List<Vector2Int> { new Vector2Int(0, 0) });

            _maes.PressPlayButton();
            _maes.GetSimulationManager().AttemptSetPlayState(Maes.UI.SimulationPlayState.FastAsPossible);

            while (_simulation.ExplorationTracker.ExploredProportion < 0.99f)
            {
                yield return null;
            }

            Assert.AreEqual(1, _minotaurs.First().GetDoorways().Count);
        }

        [Test(ExpectedResult = null)]
        public IEnumerator Corner()
        {
            InitSimulator("doorway_corner", new List<Vector2Int> { new Vector2Int(0, 0) });

            _maes.PressPlayButton();
            _maes.GetSimulationManager().AttemptSetPlayState(Maes.UI.SimulationPlayState.FastAsPossible);

            while (_simulation.ExplorationTracker.ExploredProportion < 0.99f)
            {
                yield return null;
            }

            Assert.AreEqual(1, _minotaurs.First().GetDoorways().Count);
        }

        [Test(ExpectedResult = null)]
        public IEnumerator Hallway()
        {
            InitSimulator("hallway", new List<Vector2Int> { new Vector2Int(0, -24) });

            _maes.PressPlayButton();
            _maes.GetSimulationManager().AttemptSetPlayState(Maes.UI.SimulationPlayState.FastAsPossible);

            while (_simulation.ExplorationTracker.ExploredProportion < 0.99f)
            {
                yield return null;
            }

            Assert.AreEqual(0, _minotaurs.First().GetDoorways().Count);
        }
    }
}