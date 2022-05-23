using System;
using System.Collections.Generic;
using JetBrains.Annotations;
using Maes.ExplorationAlgorithm.TheNextFrontier;
using Maes.Map;
using Maes.Map.MapGen;
using Maes.Map.Visualization;
using Maes.Robot;
using Maes.Statistics;
using Maes.UI;
using UnityEngine;

namespace Maes {
    public class Simulation : MonoBehaviour, ISimulationUnit {
        public static Simulation SingletonInstance;
        public int SimulatedLogicTicks { get; private set; } = 0;
        public int SimulatedPhysicsTicks { get; private set; } = 0;
        public float SimulateTimeSeconds { get; private set; } = 0;

        public MapGenerator MapGenerator;
        public RobotSpawner RobotSpawner;
        public ExplorationVisualizer explorationVisualizer;

        private SimulationScenario _scenario;
        private SimulationMap<bool> _collisionMap;
        public List<MonaRobot> Robots;

        [CanBeNull] private MonaRobot _selectedRobot;
        public bool HasSelectedRobot() => _selectedRobot != null;
        [CanBeNull] private VisibleTagInfoHandler _selectedTag;
        public bool HasSelectedTag() => _selectedTag != null;
        public ExplorationTracker ExplorationTracker { get; private set; }
        public CommunicationManager _communicationManager;

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
            
            Robots = scenario.RobotSpawner(_collisionMap, RobotSpawner);
            _communicationManager.SetRobotRelativeSize(scenario.RobotConstraints.AgentRelativeSize);
            foreach (var robot in Robots)
                robot.OnRobotSelected = SetSelectedRobot;
            
            _communicationManager.SetRobotReferences(Robots);

            ExplorationTracker = new ExplorationTracker(_collisionMap, explorationVisualizer, scenario.RobotConstraints);
        }

        public void SetSelectedRobot([CanBeNull] MonaRobot newSelectedRobot) {
            // Disable outline on previously selected robot
            if (_selectedRobot != null) _selectedRobot.outLine.enabled = false;
            _selectedRobot = newSelectedRobot;
            if (newSelectedRobot != null) newSelectedRobot.outLine.enabled = true;
            ExplorationTracker.SetVisualizedRobot(newSelectedRobot);
            if(_selectedRobot == null) SimInfoUIController.ClearSelectedRobot();
            UpdateDebugInfo();
        }
        
        public void SelectFirstRobot() {
            SetSelectedRobot(Robots[0]);
        }

        public void SetSelectedTag([CanBeNull] VisibleTagInfoHandler newSelectedTag) {
            if (_selectedTag != null) _selectedTag.outline.enabled = false;
            _selectedTag = newSelectedTag;
            if (newSelectedTag != null) newSelectedTag.outline.enabled = true;
            UpdateDebugInfo();
        }

        public void LogicUpdate() {
            _debugVisualizer.LogicUpdate();
            ExplorationTracker.LogicUpdate(Robots);
            Robots.ForEach(robot => robot.LogicUpdate());
            SimulatedLogicTicks++;
            _communicationManager.LogicUpdate();
        }

        public void PhysicsUpdate() {
            Robots.ForEach(simUnit => simUnit.PhysicsUpdate());
            Physics2D.Simulate(GlobalSettings.PhysicsTickDeltaSeconds);
            SimulateTimeSeconds += GlobalSettings.PhysicsTickDeltaSeconds;
            SimulatedPhysicsTicks++;
            _debugVisualizer.PhysicsUpdate();
            _communicationManager.PhysicsUpdate();
        }

        /// <summary>
        /// Tests specifically if The Next Frontier is no longer doing any work.
        /// </summary>
        public bool TnfBotsOutOfFrontiers() {
            var res = true;
            foreach (var monaRobot in Robots) {
                res &= (monaRobot.ExplorationAlgorithm as TnfExplorationAlgorithm)?.IsOutOfFrontiers() ?? true;
            }

            return res;
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
                if (GlobalSettings.IsRosMode) {
                    SimInfoUIController.UpdateAlgorithmDebugInfo(_selectedRobot.ExplorationAlgorithm.GetDebugInfo());
                    // SimInfoUIController.UpdateControllerDebugInfo(_selectedRobot.Controller.GetDebugInfo());
                }
                else {
                    SimInfoUIController.UpdateAlgorithmDebugInfo(_selectedRobot.ExplorationAlgorithm.GetDebugInfo());
                    SimInfoUIController.UpdateControllerDebugInfo(_selectedRobot.Controller.GetDebugInfo());
                }
                
            }
            if (_selectedTag != null) {
                SimInfoUIController.UpdateTagDebugInfo(_selectedTag.GetDebugInfo());
            }
        }

        public void ShowAllTags() {
            _debugVisualizer.RenderVisibleTags();
        }

        public void ShowSelectedTags() {
            if (_selectedRobot != null) {
                _debugVisualizer.RenderSelectedVisibleTags(_selectedRobot.id);
            }
        }

        public void ClearVisualTags() {
            _debugVisualizer.HideAllTags();
        }

        public void RenderCommunicationLines() {
            _debugVisualizer.RenderCommunicationLines();
        }

        public void Awake() {
            SingletonInstance = this;
        }

        public Vector2 WorldCoordinateToSlamPosition(Vector2 worldPosition) {
            return worldPosition;
        }
    }
}