using System;
using System.Collections.Generic;
using System.Linq;
using Dora.MapGeneration;
using Dora.Robot;
using Dora.Statistics;
using JetBrains.Annotations;
using UnityEngine;

namespace Dora {
    public class Simulation : MonoBehaviour, ISimulationUnit {
        public int SimulatedLogicTicks { get; private set; } = 0;
        public int SimulatedPhysicsTicks { get; private set; } = 0;
        public float SimulateTimeSeconds { get; private set; } = 0;

        public MapGenerator MapGenerator;
        public RobotSpawner RobotSpawner;
        public ExplorationVisualizer explorationVisualizer;

        private SimulationScenario _scenario;
        private SimulationMap<bool> _collisionMap;
        private List<MonaRobot> _robots;

        [CanBeNull] private MonaRobot _selectedRobot;

        public ExplorationTracker ExplorationTracker { get; private set; }
        private CommunicationManager _communicationManager;

        // The debugging visualizer provides 
        private DebuggingVisualizer _debugVisualizer = new DebuggingVisualizer();

        public SimulationInfoUIController SimInfoUIController;

        // Sets up the simulation by generating the map and spawning the robots
        public void SetScenario(SimulationScenario scenario) {
            _scenario = scenario;
            _collisionMap = scenario.MapSpawner(MapGenerator);

            _communicationManager = new CommunicationManager(_collisionMap, scenario.RobotConstraints, _debugVisualizer);
            RobotSpawner.CommunicationManager = _communicationManager;
            RobotSpawner.RobotConstraints = scenario.RobotConstraints;
            
            _robots = scenario.RobotSpawner(_collisionMap, RobotSpawner);
            foreach (var robot in _robots)
                robot.OnRobotSelected = SetSelectedRobot;
            
            _communicationManager.SetRobotReferences(_robots);

            ExplorationTracker = new ExplorationTracker(_collisionMap, explorationVisualizer, scenario.RobotConstraints);
        }

        public void SetSelectedRobot([CanBeNull] MonaRobot robot) {
            _selectedRobot = robot;
            ExplorationTracker.SetVisualizedRobot(robot);
            UpdateDebugInfo();
        }

        public void LogicUpdate() {
            _debugVisualizer.LogicUpdate();
            ExplorationTracker.LogicUpdate(_robots);
            _robots.ForEach(robot => robot.LogicUpdate());
            SimulatedLogicTicks++;
            _communicationManager.LogicUpdate();
        }

        public void PhysicsUpdate() {
            _robots.ForEach(simUnit => simUnit.PhysicsUpdate());
            Physics2D.Simulate(GlobalSettings.PhysicsTickDeltaSeconds);
            SimulateTimeSeconds += GlobalSettings.PhysicsTickDeltaSeconds;
            SimulatedPhysicsTicks++;
            _debugVisualizer.PhysicsUpdate();
            _communicationManager.PhysicsUpdate();
        }

        private void OnDrawGizmos() {
            _debugVisualizer.Render();
        }

        // ----- Future work -------
        public object SaveState() {
            throw new NotImplementedException();
        }

        public void RestoreState(object stateInfo) {
            throw new NotImplementedException();
        }

        public void UpdateDebugInfo() {
            if (_selectedRobot != null) {
                SimInfoUIController.UpdateAlgorithmDebugInfo(_selectedRobot.ExplorationAlgorithm.GetDebugInfo());
                SimInfoUIController.UpdateControllerDebugInfo(_selectedRobot.Controller.GetDebugInfo());
            }
        }
    }
}