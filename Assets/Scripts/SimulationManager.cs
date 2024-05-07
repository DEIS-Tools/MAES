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

using System;
using System.Collections.Generic;
using Maes.Statistics;
using Maes.UI;
using Maes.Utilities;
using UnityEngine;
using UnityEngine.UI;

namespace Maes {
    public class SimulationManager : MonoBehaviour {
        private SimulationPlayState _playState = SimulationPlayState.Paused;

        public GameObject SimulationPrefab;
        public Queue<SimulationScenario> _scenarios = new Queue<SimulationScenario>();

        public Queue<SimulationScenario> _initialScenarios = new Queue<SimulationScenario>();
        public SimulationSpeedController UISpeedController;
        public GameObject UIControllerDebugTitle;
        public GameObject UIControllerDebugInfo;
        public Text SimulationStatusText;
        private int _physicsTicksSinceUpdate = 0;

        public SimulationInfoUIController simulationInfoUIController;

        public SimulationScenario _currentScenario;
        public Simulation CurrentSimulation;
        private GameObject _simulationGameObject;
        
        public GameObject RosClockPrefab;
        public GameObject RosVisualizerPrefab;

        internal SimulationPlayState PlayState { get; } = SimulationPlayState.Paused;
        private int _logicTicksCurrentSim = 0;

        // Runs once when starting the program
        private void Start() {
            // This simulation handles physics updates custom time factors, so disable built in real time physics calls
            Physics.autoSimulation = false;
            Physics2D.simulationMode = SimulationMode2D.Script;
            
            // Adapt UI for ros mode
            if (GlobalSettings.IsRosMode) {
                CreateRosClockAndVisualiserObjects();
                RemoveFastForwardButtonsFromControlPanel();
                UIControllerDebugTitle.SetActive(false);
                UIControllerDebugInfo.SetActive(false);
            }
            UISpeedController.UpdateButtonsUI(SimulationPlayState.Play);
        }

        public void RemoveFastForwardButtonsFromControlPanel() {
            // Deactivate fast forward buttons
            UISpeedController.stepperButton.gameObject.SetActive(false);
            UISpeedController.fastForwardButton.gameObject.SetActive(false);
            UISpeedController.fastAsPossibleButton.gameObject.SetActive(false);
            
            // Resize background
            var controlPanel = GameObject.Find("ControlPanel");
            var cpRectTransform = controlPanel.GetComponent<RectTransform>();
            cpRectTransform.sizeDelta = new Vector2(100, 50);
            
            // Reposition play button
            var playButton = GameObject.Find("PlayButton");
            var pbRectTransform = playButton.GetComponent<RectTransform>();
            pbRectTransform.anchoredPosition = new Vector2(-20, 0);
            
            // Reposition pause button
            var pauseButton = GameObject.Find("PauseButton");
            var pauseRectTransform = pauseButton.GetComponent<RectTransform>();
            pauseRectTransform.anchoredPosition = new Vector2(20, 0);
        }

        public void CreateRosClockAndVisualiserObjects() {
            Instantiate(RosClockPrefab, new Vector3(0, 0, 0), Quaternion.identity);
            Instantiate(RosVisualizerPrefab, new Vector3(0, 0, 0), Quaternion.identity);
        }

        internal SimulationPlayState AttemptSetPlayState(SimulationPlayState targetState) {
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


        private void Update()
        {
            if (Input.GetKey(KeyCode.LeftShift) || Input.GetKey(KeyCode.RightShift)) {
                if (Input.GetKeyDown(KeyCode.Alpha1))
                {
                    AttemptSetPlayState(SimulationPlayState.Play);
                } 
                else if (Input.GetKeyDown(KeyCode.Alpha2))
                {
                    AttemptSetPlayState(SimulationPlayState.FastForward);
                }
                else if (Input.GetKeyDown(KeyCode.Alpha3))
                {
                    AttemptSetPlayState(SimulationPlayState.FastAsPossible);
                }
                else if (Input.GetKeyDown(KeyCode.P))
                {
                    AttemptSetPlayState(SimulationPlayState.Step);
                }
            }
        }

        // Timing variables for controlling the simulation in a manner that is decoupled from Unity's update system
        private long _nextUpdateTimeMillis = 0;

        // This method is responsible for executing simulation updates at an appropriate speed, to provide simulation in
        // real time (or whatever speed setting is chosen)
        private void FixedUpdate() {
            if (_playState == SimulationPlayState.Paused) {
                if (Application.isBatchMode) {
                    // The simulation will only enter paused mode after finishing when in headless/batch mode
                    Application.Quit(0);
                }
                return;
            }

            if (Application.isBatchMode && _playState != SimulationPlayState.FastAsPossible)
            {
                AttemptSetPlayState(SimulationPlayState.FastAsPossible);
            }

            long startTimeMillis = DateTime.Now.Ticks / TimeSpan.TicksPerMillisecond;
            int millisPerFixedUpdate = (int) (1000f * Time.fixedDeltaTime);
            // Subtract 8 milliseconds to allow for other procedures such as rendering to occur between updates 
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
            var csvWriter = new StatisticsCSVWriter(CurrentSimulation,$"{_currentScenario.StatisticsFileName}");
            csvWriter.CreateCSVFile(",");
        }

        // Calls update on all children of SimulationContainer that are of type SimulationUnit
        private bool UpdateSimulation() {
            if (_currentScenario != null && _currentScenario.HasFinishedSim(CurrentSimulation)) {
                if (GlobalSettings.ShouldWriteCSVResults && _currentScenario.HasFinishedSim(CurrentSimulation))
                    CreateStatisticsFile();
                if (_scenarios.Count != 0) { //If last simulation, let us keep looking around in it
                    RemoveCurrentSimulation();
                }
            }

            if (_currentScenario == null) {
                if (_scenarios.Count == 0) {
                    // Indicate that no further updates are needed
                    return false;
                }

                // Otherwise continue to next simulation in the queue
                CreateSimulation(_scenarios.Dequeue());
            }

            CurrentSimulation.PhysicsUpdate();
            _physicsTicksSinceUpdate++;
            var shouldContinueSim = true;
            if (_physicsTicksSinceUpdate >= GlobalSettings.PhysicsTicksPerLogicUpdate) {
                CurrentSimulation.LogicUpdate();
                _logicTicksCurrentSim++;
                _physicsTicksSinceUpdate = 0;
                if(GlobalSettings.ShouldWriteCSVResults 
                   && _logicTicksCurrentSim != 0 
                   && _logicTicksCurrentSim % GlobalSettings.TicksPerStatsSnapShot == 0) 
                    CurrentSimulation.ExplorationTracker.CreateSnapShot();
                UpdateStatisticsUI();
                
                
                // If the simulator is in step mode, then automatically pause after logic step has been performed
                if (_playState == SimulationPlayState.Step)
                    shouldContinueSim = false;
            }

            var simulatedTimeSpan = TimeSpan.FromSeconds(CurrentSimulation.SimulateTimeSeconds);
            var output = simulatedTimeSpan.ToString(@"hh\:mm\:ss");
            SimulationStatusText.text = "Phys. ticks: " + CurrentSimulation.SimulatedPhysicsTicks +
                                        "\nLogic ticks: " + CurrentSimulation.SimulatedLogicTicks +
                                        "\nSimulated: " + output;
            
            return shouldContinueSim;
        }

        public void CreateSimulation(SimulationScenario scenario) {
            _currentScenario = scenario;
            _simulationGameObject = Instantiate(SimulationPrefab, transform);
            CurrentSimulation = _simulationGameObject.GetComponent<Simulation>();
            CurrentSimulation.SetScenario(scenario);
            CurrentSimulation.SimInfoUIController = simulationInfoUIController;
            _logicTicksCurrentSim = 0;

            simulationInfoUIController.NotifyNewSimulation(CurrentSimulation);
        }


        private void UpdateStatisticsUI() {
            simulationInfoUIController.UpdateStatistics(CurrentSimulation);
            if (CurrentSimulation != null)
                CurrentSimulation.UpdateDebugInfo();
        }

        public void RemoveCurrentSimulation() {
            Destroy(_simulationGameObject);
            _currentScenario = null;
            CurrentSimulation = null;
            _simulationGameObject = null;
            _logicTicksCurrentSim = 0;
        }

        public Simulation GetCurrentSimulation() {
            return CurrentSimulation;
        }

        public void EnqueueScenario(SimulationScenario simulationScenario) {
            if (HasActiveScenario()) {
                _scenarios.Enqueue(simulationScenario);
            } else // This is the first scenario, initialize it immediately 
                CreateSimulation(simulationScenario);
        }

        public bool HasActiveScenario() {
            return _currentScenario is not null;
        }
        
    }
}