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
            UpdateVersionNumberText();
        }

        public void UpdateVersionNumberText() {
            var versionNumberText = GameObject.Find("VersionNumber").GetComponent<Text>();
            versionNumberText.text = "v." + PlayerSettings.bundleVersion;
        }
        
        public static Simulator GetInstance() {
            return _instance ??= new Simulator();
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
            if (!GlobalSettings.IsRosMode) StartSimulation();
        }

        public void EnqueueScenario(SimulationScenario scenario) {
            _simulationManager.EnqueueScenario(scenario);
        }
        public void EnqueueScenarios(IEnumerable<SimulationScenario> scenario) {
            foreach (var simulationScenario in scenario)
                _simulationManager.EnqueueScenario(simulationScenario);
        }
        
        public void StartSimulation() {
            if (_simulationManager.PlayState == SimulationPlayState.Play)
                throw new InvalidOperationException("Cannot start simulation when it is already in play mode");
            if (!_simulationManager.HasActiveScenario())
                throw new InvalidOperationException("You must enqueue at least one scenario before starting the" +
                                                    " simulation");


            _simulationManager.AttemptSetPlayState(SimulationPlayState.Play);
        }
    }
}