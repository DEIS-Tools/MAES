using System.Collections;
using System.Collections.Generic;
using Maes;
using Maes.Robot;
using NUnit.Framework;
using UnityEngine;
using Maes.ExplorationAlgorithm.Minotaur;
using Maes.Utilities.Files;
using System.Linq;
using Maes.Map.MapGen;

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
        private RobotConstraints GetRobotConstraints()
        {
            return new RobotConstraints(
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
        }
        private void InitSimulator(string mapName, List<Vector2Int> robotSpawnPositions)
        {
            var constraints = GetRobotConstraints();
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
        private void InitSimulator(BuildingMapConfig mapConfig, List<Vector2Int> robotSpawnPositions)
        {
            var constraints = GetRobotConstraints();
            var testingScenario = new SimulationScenario(RandomSeed,
                mapSpawner: generator => generator.GenerateMap(mapConfig),
                hasFinishedSim: simulation => false,
                robotConstraints: constraints,
                robotSpawner: (map, spawner) => spawner.SpawnRobotsAtPositions(robotSpawnPositions, map, RandomSeed, 1,
                    robotSeed =>
                    {
                        var algorithm = new MinotaurDoorwayMock(constraints, RandomSeed, 2);
                        _minotaurs.Add(algorithm);
                        return algorithm;
                    }));

            _maes = Simulator.GetInstance();
            _maes.EnqueueScenario(testingScenario);
            _simulation = _maes.GetSimulationManager().CurrentSimulation;
        }

        private IEnumerator AssertDoorsWhenFinished(int doorAmount)
        {
            if (_simulation.SimulatedLogicTicks > 36000)
                yield return false;
            while (_simulation.ExplorationTracker.ExploredProportion < 0.999f)
            {
                yield return null;
            }

            Assert.AreEqual(doorAmount, _minotaurs.First().GetDoorways().Count);
        }

        [Test(ExpectedResult = null)]
        public IEnumerator BlankMap()
        {
            InitSimulator("blank", new List<Vector2Int> { new Vector2Int(0, 0) });

            _maes.PressPlayButton();
            _maes.GetSimulationManager().AttemptSetPlayState(Maes.UI.SimulationPlayState.FastAsPossible);
            return AssertDoorsWhenFinished(0);
        }

        [Test(ExpectedResult = null)]
        public IEnumerator SingleDoorway()
        {
            InitSimulator("doorway", new List<Vector2Int> { new Vector2Int(0, 0) });

            _maes.PressPlayButton();
            _maes.GetSimulationManager().AttemptSetPlayState(Maes.UI.SimulationPlayState.FastAsPossible);
            return AssertDoorsWhenFinished(1);
        }

        [Test(ExpectedResult = null)]
        public IEnumerator Corner()
        {
            InitSimulator("doorway_corner", new List<Vector2Int> { new Vector2Int(0, 0) });

            _maes.PressPlayButton();
            _maes.GetSimulationManager().AttemptSetPlayState(Maes.UI.SimulationPlayState.FastAsPossible);
            return AssertDoorsWhenFinished(1);
        }

        [Test(ExpectedResult = null)]
        public IEnumerator Hallway()
        {
            InitSimulator("hallway", new List<Vector2Int> { new Vector2Int(0, -24) });

            _maes.PressPlayButton();
            _maes.GetSimulationManager().AttemptSetPlayState(Maes.UI.SimulationPlayState.FastAsPossible);
            return AssertDoorsWhenFinished(1);
        }

        [Test(ExpectedResult = null)]
        public IEnumerator CompleteMap()
        {
            var buildingConfig = new BuildingMapConfig(RandomSeed, widthInTiles: 100, heightInTiles: 100);
            InitSimulator(buildingConfig, new List<Vector2Int> { new Vector2Int(0, 0) });

            _maes.PressPlayButton();
            _maes.GetSimulationManager().AttemptSetPlayState(Maes.UI.SimulationPlayState.FastAsPossible);
            if (_simulation.SimulatedLogicTicks > 36000)
                yield return false;
            while (_simulation.ExplorationTracker.ExploredProportion < 0.999f)
            {
                yield return null;
            }
            Assert.GreaterOrEqual(0.9999f, _simulation.ExplorationTracker.ExploredProportion);
        }
    }
}