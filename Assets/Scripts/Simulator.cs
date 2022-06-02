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
using System.Collections;
using System.Collections.Generic;
using Maes.UI;
using UnityEditor;
using UnityEngine;
using UnityEngine.UI;
using Object = UnityEngine.Object;

namespace Maes {
    public class Simulator {

        private static Simulator _instance = null;
        private GameObject _maesGameObject;
        private SimulationManager _simulationManager;

        private Simulator() {
            // Initialize the simulator by loading the prefab from the resources and then instantiating the prefab
            var prefab = Resources.Load("MAES", typeof(GameObject)) as GameObject;
            _maesGameObject = Object.Instantiate(prefab);
            _simulationManager = _maesGameObject.GetComponentInChildren<SimulationManager>();
        }

        public static Simulator GetInstance() {
            return _instance ??= new Simulator();
        }

        // Clears the singleton instance and removes the simulator game object
        public static void Destroy() {
            if (_instance != null) {
                Object.Destroy(_instance._maesGameObject);
                _instance = null;
            }
        }
        
        /// <summary>
        /// This method is used to start the simulation in a predefined configuration that will change depending on
        /// whether the simulation is in ros mode or not.
        /// </summary>
        public void DefaultStart(bool isRosMode = false) {
            GlobalSettings.IsRosMode = isRosMode;
            IEnumerable<SimulationScenario> generatedScenarios;
            if (GlobalSettings.IsRosMode) {
                generatedScenarios = ScenarioGenerator.GenerateROS2Scenario();
            } else {
                generatedScenarios = ScenarioGenerator.GenerateYoutubeVideoScenarios();
            }
            EnqueueScenarios(generatedScenarios);
            if (Application.isBatchMode) {
                _simulationManager.AttemptSetPlayState(SimulationPlayState.FastAsPossible);
            }
        }

        public void EnqueueScenario(SimulationScenario scenario) {
            _simulationManager.EnqueueScenario(scenario);
        }
        public void EnqueueScenarios(IEnumerable<SimulationScenario> scenario) {
            foreach (var simulationScenario in scenario)
                _simulationManager.EnqueueScenario(simulationScenario);
        }
        
        public void PressPlayButton() {
            if (_simulationManager.PlayState == SimulationPlayState.Play)
                throw new InvalidOperationException("Cannot start simulation when it is already in play mode");
            if (!_simulationManager.HasActiveScenario())
                throw new InvalidOperationException("You must enqueue at least one scenario before starting the" +
                                                    " simulation");


            _simulationManager.AttemptSetPlayState(SimulationPlayState.Play);
        }

        public SimulationManager GetSimulationManager() {
            return _simulationManager;
        }
    }
}