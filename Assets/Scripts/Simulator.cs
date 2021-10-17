using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading;
using Dora.MapGeneration;
using Dora.Robot;
using Dora.Statistics;
using Dora.Utilities;
using UnityEngine;
using UnityEngine.UI;

namespace Dora
{
    public class Simulator : MonoBehaviour
    {
        public SimulationConfiguration SimConfig = new SimulationConfiguration(); // Should this be a struct?!
        private SimulationPlayState _playState = SimulationPlayState.Paused;

        public Transform simulationContainer;
        public MapGenerator MapGenerator;
        public RobotSpawner RobotSpawner;

        private ExplorationTracker _explorationTracker;
        public ExplorationVisualizer explorationVisualizer;
        private List<MonaRobot> _robots;

        public Text TestingText;
        private int _physicsTickCount = 0;
        private int _robotLogicTickCount = 0;
        private int _physicsTicksSinceUpdate = 0;
        private int _simulatedTimeMillis = 0;

        public SimulationPlayState PlayState { get; }

        public SimulationPlayState AttemptSetPlayState(SimulationPlayState targetState)
        {
            if (targetState == _playState) return _playState;
            // TODO: Check if possible to change (For example, not possible if no map is generated)
            _playState = targetState;
            // Reset next update time when changing play mode to avoid skipping ahead
            _nextUpdateTimeMillis = TimeUtils.CurrentTimeMillis();
            return _playState;
        }

        private void Start()
        {
            Physics.autoSimulation = false;
            Physics2D.simulationMode = SimulationMode2D.Script;
            GenerateSimulation();
        }

        private void GenerateSimulation()
        {
            
            var config = new CaveMapConfig(100,
                100,
                (int)new DateTimeOffset(DateTime.Now).ToUnixTimeSeconds(),
                4,
                2,
                48,
                10,
                1,
                1,
                2);
            var collisionMap = MapGenerator.GenerateCaveMap(config, 
                3.0f,
                true);
            
            //var officeConfig = new OfficeMapConfig(60, 60,  (int)new DateTimeOffset(DateTime.Now).ToUnixTimeSeconds(), 8, 3, 5, 2, 0, 65, 2, 2.0f);
            //var collisionMap = MapGenerator.GenerateOfficeMap(officeConfig, 3.0f, true);
            
            _robots = RobotSpawner.SpawnRobots();
            _explorationTracker = new ExplorationTracker(collisionMap, explorationVisualizer);
        }

        // Timing variables for controlling the simulation in a manner that is decoupled from Unity's update system
        private long _nextUpdateTimeMillis = 0;

        private void FixedUpdate()
        {
            if (_playState == SimulationPlayState.Paused) return;

            long startTimeMillis = DateTime.Now.Ticks / TimeSpan.TicksPerMillisecond;
            int millisPerFixedUpdate = (int) (1000f * Time.fixedDeltaTime);
            // Subtract 5 milliseconds to allow for other procedures such as rendering to occur between updates 
            millisPerFixedUpdate -= 5;
            long fixedUpdateEndTime = startTimeMillis + millisPerFixedUpdate;

            // ReSharper disable once PossibleLossOfFraction
            int physicsTickDeltaMillis = SimConfig.LogicTickDeltaMillis / SimConfig.PhysicsTicksPerLogicUpdate;

            // Only calculate updates if there is still time left in the current update
            while (TimeUtils.CurrentTimeMillis() - startTimeMillis < millisPerFixedUpdate)
            {
                // Yield if no more updates are needed this FixedUpdate cycle
                if (_nextUpdateTimeMillis > fixedUpdateEndTime) break;

                UpdateSimulation(physicsTickDeltaMillis, SimConfig);
                
                // The delay before simulating the next update is dependant on the current simulation play speed
                int updateDelayMillis = physicsTickDeltaMillis / (int) _playState;
                _nextUpdateTimeMillis = _nextUpdateTimeMillis + updateDelayMillis;
                // Do not try to catch up if more than 0.5 seconds behind (higher if tick delta is high)
                long maxDelayMillis = Math.Max(500, physicsTickDeltaMillis * 10);
                _nextUpdateTimeMillis = Math.Max(_nextUpdateTimeMillis, TimeUtils.CurrentTimeMillis() - maxDelayMillis);
            }
            var simulatedTimeSpan = TimeSpan.FromMilliseconds(_simulatedTimeMillis);
            var output = simulatedTimeSpan.ToString(@"hh\:mm\:ss");
            var following = CameraController.SingletonInstance.movementTransform == null
                ? ""
                : "\nPress Esc to release camera";
            TestingText.text = "Phys. ticks: " + _physicsTickCount + 
                        "\nLogic ticks: " + _robotLogicTickCount + 
                        "\nSimulated: " + output + following;
        }

        // Calls update on all children of SimulationContainer that are of type SimulationUnit
        private void UpdateSimulation(int physicsTickDeltaMillis, SimulationConfiguration config)
        {
            List<ISimulationUnit> simUnits = new List<ISimulationUnit>();

            AddAllSimulationUnitsInChildren(simulationContainer, simUnits);
            simUnits.ForEach(simUnit => simUnit.PhysicsUpdate(config));
            
            float physicsTickDeltaSeconds = physicsTickDeltaMillis / 1000.0f;
            //Physics.Simulate(physicsTickDeltaSeconds);
            Physics2D.Simulate(physicsTickDeltaSeconds);
            _physicsTickCount += 1; 
            _simulatedTimeMillis += physicsTickDeltaMillis;
            _physicsTicksSinceUpdate++;
            
            _explorationTracker.Update(config, _robots);
            
            if (_physicsTicksSinceUpdate >= SimConfig.PhysicsTicksPerLogicUpdate)
            {
                simUnits.ForEach(simUnit => simUnit.LogicUpdate(config));
                _physicsTicksSinceUpdate = 0;
                _robotLogicTickCount += 1;
            }
        }

        // Recursively finds all children that are of type SimulationUnit
        // Children are added depth first 
        private void AddAllSimulationUnitsInChildren(Transform parent, List<ISimulationUnit> simUnits)
        {
            foreach (Transform child in parent)
            {
                AddAllSimulationUnitsInChildren(child, simUnits);
                ISimulationUnit unit = child.GetComponent<ISimulationUnit>();
                if (unit != null) simUnits.Add(unit);
            }
        }
    }
}