using System;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

namespace Maes.UI {
    public class SimulationInfoUIController : MonoBehaviour {
        public Image ExplorationBarMask, CoverageBarMask;
        public Text ProgressPercentageText, CoveragePercentageText;
        public Text ExplorationRateText;

        public Text AlgorithmDebugText;
        public Text ControllerDebugText;

        public Button AllExplorationButton;
        public Button AllCoverageButton;
        public Button AllExplorationHeatMapButton;
        public Button AllCoverageHeatMapButton;

        private List<Button> _mapVisualizationToggleGroup;
        private Color _mapVisualizationColor = Color.white;
        private Color _mapVisualizationSelectedColor = new Color(150 / 255f, 200 / 255f, 150 / 255f);

        public Simulator simulator;
        // Represents a function that modifies the given simulation in some way
        // (for example by changing map visualization mode)
        delegate void SimulationModification(Simulation? simulation);

        private SimulationModification? _mostRecentMapVisualizationModification;

        private void Start() {
            _mapVisualizationToggleGroup = new List<Button>() {AllExplorationButton, AllCoverageButton, AllExplorationHeatMapButton, AllCoverageHeatMapButton};
            SelectVisualizationButton(AllExplorationButton);
            
            // Set listeners for all map visualization buttons
            AllExplorationButton.onClick.AddListener(() => {
                SelectVisualizationButton(AllExplorationButton);
                ExecuteAndRememberMapVisualizationModification((sim) => sim?.ExplorationTracker.ShowAllRobotExploration());
            });
            
            AllCoverageButton.onClick.AddListener(() => {
                SelectVisualizationButton(AllCoverageButton);
                ExecuteAndRememberMapVisualizationModification((sim) => sim?.ExplorationTracker.ShowAllRobotCoverage());
            });
            
            AllExplorationHeatMapButton.onClick.AddListener(() => {
                SelectVisualizationButton(AllExplorationHeatMapButton);
                ExecuteAndRememberMapVisualizationModification((sim) => sim?.ExplorationTracker.ShowAllRobotExplorationHeatMap());
            });
            
            AllCoverageHeatMapButton.onClick.AddListener(() => {
                SelectVisualizationButton(AllCoverageHeatMapButton);
                ExecuteAndRememberMapVisualizationModification((sim) => sim?.ExplorationTracker.ShowAllRobotCoverageHeatMap());
            });
        }

        // This function executes the given map visualization change and remembers it.
        // Whenever the simulator creates a new simulation the most recent visualization change is repeated 
        private void ExecuteAndRememberMapVisualizationModification(SimulationModification modificationFunc) {
            _mostRecentMapVisualizationModification = modificationFunc;
            modificationFunc(simulator.CurrentSimulation);
        }

        // Highlights the selected map visualization button
        private void SelectVisualizationButton(Button selectedButton) {
            foreach (var button in _mapVisualizationToggleGroup) 
                button.image.color = _mapVisualizationColor;

            selectedButton.image.color = _mapVisualizationSelectedColor;
        }

        public void SetExplorationProgress(float progress) {
            ExplorationBarMask.fillAmount = progress;
            ProgressPercentageText.text = (progress * 100f).ToString("#.00") + "%";
        }

        public void SetCoverageProgress(float progress) {
            CoverageBarMask.fillAmount = progress;
            CoveragePercentageText.text = (progress * 100f).ToString("#.00") + "%";            
        }

        public void UpdateStatistics(Simulation currentSimulation) {
            SetExplorationProgress(currentSimulation.ExplorationTracker.ExploredProportion);
            SetCoverageProgress(currentSimulation.ExplorationTracker.CoverageProportion);
            ExplorationRateText.text = "Exploration rate (cells/minute): " +
                                       (currentSimulation.ExplorationTracker.ExploredTriangles /
                                        currentSimulation.SimulateTimeSeconds).ToString("#.0") + "\n" +
                                       "Coverage rate (cells/minute): " +
                                       (currentSimulation.ExplorationTracker.CoveredMiniTiles * 2 /
                                        currentSimulation.SimulateTimeSeconds).ToString("#.0");
            // Covered tiles multiplied by two to convert from mini-tiles to triangles/cells ^
        }

        public void UpdateAlgorithmDebugInfo(string info) {
            AlgorithmDebugText.text = info;
        }


        public void UpdateControllerDebugInfo(string info) {
            ControllerDebugText.text = info;
        }

        // Called whenever the simulator instantiates a new simulation object 
        public void NotifyNewSimulation(Simulation? newSimulation) {
            _mostRecentMapVisualizationModification?.Invoke(newSimulation);
        }
    }
}