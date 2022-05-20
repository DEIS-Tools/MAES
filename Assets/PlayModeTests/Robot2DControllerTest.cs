using System.Collections;
using Maes;
using Maes.ExplorationAlgorithm;
using Maes.ExplorationAlgorithm.RandomBallisticWalk;
using Maes.Map.MapGen;
using Maes.Robot;
using Maes.Robot.Task;
using NUnit.Framework;
using UnityEngine;
using UnityEngine.TestTools;

namespace PlayModeTests {
    public class Robot2DControllerTest {

        private const int RandomSeed = 123;
        private Simulator _maes;
        private RobotTestAlgorithm _testAlgorithm;
        private Simulation _simulation;
        private MonaRobot _robot;

        [SetUp]
        public void InitializeTestingSimulator() {
            var mapConfiguration = new CaveMapConfig(randomSeed: RandomSeed, 
                widthInTiles: 50, 
                heightInTiles: 50, 
                smoothingRuns: 4,
                connectionPassagesWidth: 4, 
                randomFillPercent: 0, 
                wallThresholdSize: 10, 
                roomThresholdSize: 10,
                borderSize: 1);
            var robotConstraints = new RobotConstraints();
            var testingScenario = new SimulationScenario(RandomSeed, 
                mapSpawner: generator => generator.GenerateCaveMap(mapConfiguration),
                hasFinishedSim: simulation => false,
                robotSpawner: (map, spawner) => spawner.SpawnRobotsTogether( map, RandomSeed, 1, 
                    Vector2Int.zero, (robotSeed) => {
                        var algorithm = new RobotTestAlgorithm();
                        _testAlgorithm = algorithm;
                        return algorithm;
                    }));
            
            _maes = Simulator.GetInstance();
            _maes.EnqueueScenario(testingScenario);
            _simulation = _maes.GetSimulationManager().CurrentSimulation;
            _robot = _simulation.Robots[0];
        }

        private class RobotTestAlgorithm : IExplorationAlgorithm {
            public int tick = 0;
            public Robot2DController _controller;

            public delegate void CustomUpdateFunction(int tick, Robot2DController controller);

            private CustomUpdateFunction onUpdate;
            public RobotTestAlgorithm(CustomUpdateFunction onUpdate) {
                this.onUpdate = onUpdate;
            }
            public object SaveState() { throw new System.NotImplementedException(); }
            public void RestoreState(object stateInfo) { }

            public void UpdateLogic() {
                if (tick == 0) 
                    _controller.Move(TargetMovementDistance);
                tick++;
            }

            public void SetController(Robot2DController controller) {
                this._controller = controller;
            }

            public string GetDebugInfo() {
                return "";
            }
        }

        [TearDown]
        public void ClearSimulator() {
            Simulator.Destroy();
        }

        
        // Test that the robot is able to move the given distance
        [UnityTest]
        [TestCase(1.0f, ExpectedResult = (IEnumerator) null)]
        [TestCase(2.0f, ExpectedResult = (IEnumerator) null)]
        [TestCase(5.0f, ExpectedResult = (IEnumerator) null)]
        [TestCase(10.0f, ExpectedResult = (IEnumerator) null)]
        [TestCase(20.0f, ExpectedResult = (IEnumerator) null)]
        public IEnumerator MoveTo_IsDistanceCorrectTest(float movementDistance) {
            
            _testAlgorithm.TargetMovementDistance = movementDistance;
            var controller = _robot.Controller;

            // Register the starting position and calculate the expected position
            var transform = _robot.transform;
            var startingPosition = transform.position;
            var expectedEndingPosition = startingPosition + (controller.GetForwardDirectionVector() * movementDistance);
            
            _maes.StartSimulation();
            
            // Wait until the robot has started and completed the movement task
            while (_testAlgorithm.tick < 10 || _testAlgorithm._controller.GetStatus() != RobotStatus.Idle) {
                yield return null;
            }
            
            //  Wait 1 second (10 ticks) for the robot to stand completely still
            var movementTaskEndTick = _simulation.SimulatedLogicTicks;
            const int ticksToWait = 10;
            while (_simulation.SimulatedLogicTicks < movementTaskEndTick + ticksToWait) yield return null;

            // Assert that the actual final position approximately matches the expected final position
            var endingPosition = _robot.transform.position;
            const float maximumDeviation = 0.1f;
            var targetPositionDelta = (expectedEndingPosition - endingPosition).magnitude;
            Debug.Log($"Actual: {endingPosition}  vs  expected: {expectedEndingPosition}");
            Assert.LessOrEqual(targetPositionDelta, maximumDeviation);
        }

        [UnityTest]
        [TestCase(1.0f, ExpectedResult = (IEnumerator) null)]
        [TestCase(2.0f, ExpectedResult = (IEnumerator) null)]
        [TestCase(5.0f, ExpectedResult = (IEnumerator) null)]
        [TestCase(10.0f, ExpectedResult = (IEnumerator) null)]
        [TestCase(20.0f, ExpectedResult = (IEnumerator) null)]
        public IEnumerator Rotate_IsRotationCorrectAmount(float targetRotation) {
            
            yield return null;
            
        }
    }
}
