using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Threading;
using Dora.MapGeneration;
using Dora.Robot;
using Dora.Statistics;
using Dora.Utilities;
using UnityEngine;
using UnityEngine.UI;

namespace Dora {
    public class Simulator : MonoBehaviour {
        private SimulationPlayState _playState = SimulationPlayState.Paused;

        public GameObject SimulationPrefab;
        private Queue<SimulationScenario> _scenarios;

        public SimulationSpeedController UISpeedController;
        public Text SimulationStatusText;
        private int _physicsTicksSinceUpdate = 0;

        public SimulationInfoUIController simulationInfoUIController;

        private SimulationScenario _currentScenario;
        private Simulation _currentSimulation;
        private GameObject _simulationGameObject;

        public SimulationPlayState PlayState { get; }
        private int _logicTicksCurrentSim = 0;

        // Runs once when starting the program
        private void Start() {
            // This simulation handles physics updates custom time factors, so disable built in real time physics calls
            Physics.autoSimulation = false;
            Physics2D.simulationMode = SimulationMode2D.Script;

            _scenarios = ScenarioGenerator.GenerateArticleScenarios(1);
            CreateSimulation(_scenarios.Dequeue());
            
        }

        public SimulationPlayState AttemptSetPlayState(SimulationPlayState targetState) {
            if (targetState == _playState) return _playState;

            if (_currentScenario == null) {
                targetState = SimulationPlayState.Paused;
            }

            _playState = targetState;
            // Reset next update time when changing play mode to avoid skipping ahead
            _nextUpdateTimeMillis = TimeUtils.CurrentTimeMillis();
            UISpeedController.UpdateButtonsUI(_playState);
            return _playState;
        }


        // Timing variables for controlling the simulation in a manner that is decoupled from Unity's update system
        private long _nextUpdateTimeMillis = 0;

        // This method is responsible for executing simulation updates at an appropriate speed, to provide simulation in
        // real time (or whatever speed setting is chosen)
        private void FixedUpdate() {
            if (_playState == SimulationPlayState.Paused) return;

            long startTimeMillis = DateTime.Now.Ticks / TimeSpan.TicksPerMillisecond;
            int millisPerFixedUpdate = (int) (1000f * Time.fixedDeltaTime);
            // Subtract 5 milliseconds to allow for other procedures such as rendering to occur between updates 
            millisPerFixedUpdate -= 8;
            long fixedUpdateEndTime = startTimeMillis + millisPerFixedUpdate;

            // ReSharper disable once PossibleLossOfFraction
            int physicsTickDeltaMillis =
                GlobalSettings.LogicTickDeltaMillis / GlobalSettings.PhysicsTicksPerLogicUpdate;

            // Only calculate updates if there is still time left in the current update
            while (TimeUtils.CurrentTimeMillis() - startTimeMillis < millisPerFixedUpdate) {
                // Yield if no more updates are needed this FixedUpdate cycle
                if (_nextUpdateTimeMillis > fixedUpdateEndTime) break;

                var shouldContinue = UpdateSimulation();
                if (!shouldContinue) {
                    AttemptSetPlayState(SimulationPlayState.Paused);
                    return;
                }

                // The delay before simulating the next update is dependant on the current simulation play speed
                int updateDelayMillis = physicsTickDeltaMillis / (int) _playState;
                _nextUpdateTimeMillis = _nextUpdateTimeMillis + updateDelayMillis;
                // Do not try to catch up if more than 0.5 seconds behind (higher if tick delta is high)
                long maxDelayMillis = Math.Max(500, physicsTickDeltaMillis * 10);
                _nextUpdateTimeMillis = Math.Max(_nextUpdateTimeMillis, TimeUtils.CurrentTimeMillis() - maxDelayMillis);
            }
        }

        private void CreateStatisticsFile() {
            var csvWriter = new StatisticsCSVWriter(_currentSimulation, _currentScenario.StatisticsFileName);
            csvWriter.CreateCSVFile(",");
        }

        // Calls update on all children of SimulationContainer that are of type SimulationUnit
        private bool UpdateSimulation() {
            if (_currentScenario != null && _currentScenario.HasFinishedSim(_currentSimulation)) {
                if (GlobalSettings.ShouldWriteCSVResults && _currentScenario.HasFinishedSim(_currentSimulation))
                    CreateStatisticsFile();
                RemoveCurrentSimulation();
            }

            if (_currentScenario == null) {
                if (_scenarios.Count == 0) {
                    // Indicate that no further updates are needed
                    return false;
                }

                // Otherwise continue to next simulation in the queue
                CreateSimulation(_scenarios.Dequeue());
            }

            _currentSimulation.PhysicsUpdate();
            _physicsTicksSinceUpdate++;
            var shouldContinueSim = true;
            if (_physicsTicksSinceUpdate >= GlobalSettings.PhysicsTicksPerLogicUpdate) {
                _currentSimulation.LogicUpdate();
                _logicTicksCurrentSim++;
                _physicsTicksSinceUpdate = 0;
                if(GlobalSettings.ShouldWriteCSVResults 
                   && _logicTicksCurrentSim != 0 
                   && _logicTicksCurrentSim % GlobalSettings.TicksPerStatsSnapShot == 0) 
                    _currentSimulation.ExplorationTracker.CreateSnapShot();
                UpdateStatisticsUI();
                
                
                // If the simulator is in step mode, then automatically pause after logic step has been performed
                if (_playState == SimulationPlayState.Step)
                    shouldContinueSim = false;
            }

            var simulatedTimeSpan = TimeSpan.FromSeconds(_currentSimulation.SimulateTimeSeconds);
            var output = simulatedTimeSpan.ToString(@"hh\:mm\:ss");
            SimulationStatusText.text = "Phys. ticks: " + _currentSimulation.SimulatedPhysicsTicks +
                                        "\nLogic ticks: " + _currentSimulation.SimulatedLogicTicks +
                                        "\nSimulated: " + output;
            
            return shouldContinueSim;
        }

        public void CreateSimulation(SimulationScenario scenario) {
            _currentScenario = scenario;
            _simulationGameObject = Instantiate(SimulationPrefab, transform);
            _currentSimulation = _simulationGameObject.GetComponent<Simulation>();
            _currentSimulation.SetScenario(scenario);
            _currentSimulation.SimInfoUIController = simulationInfoUIController;
            _logicTicksCurrentSim = 0;
        }


        private void UpdateStatisticsUI() {
            simulationInfoUIController.UpdateStatistics(_currentSimulation);
            if (_currentSimulation != null)
                _currentSimulation.UpdateDebugInfo();
        }

        public void RemoveCurrentSimulation() {
            Destroy(_simulationGameObject);
            _currentScenario = null;
            _currentSimulation = null;
            _simulationGameObject = null;
            _logicTicksCurrentSim = 0;
        }

        public Simulation GetCurrentSimulation() {
            return _currentSimulation;
        }
    }
}