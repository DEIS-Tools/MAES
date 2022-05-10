using System;
using System.Collections.Generic;
using JetBrains.Annotations;
using Maes.Map.Visualization;
using UnityEngine;
using UnityEngine.Serialization;
using UnityEngine.UI;

namespace Maes.UI {
    public class SimulationInfoUIController : MonoBehaviour {
        public Image ExplorationBarMask, CoverageBarMask;
        public Text ProgressPercentageText, CoveragePercentageText;
        public Text ExplorationRateText;

        public Text AlgorithmDebugText;
        public Text ControllerDebugText;
        public Text TagDebugText;
        
        public Button AllExplorationButton;
        public Button AllCoverageButton;
        public Button AllExplorationHeatMapButton;
        public Button AllCoverageHeatMapButton;
        public Button SelectVisibleAreaButton;
        public Button SelectedSlamMapButton;
        
        public Button AllVisualizeTagsButton;
        private bool _visualizingAllTags = false;
        public Button VisualizeTagsButton;
        private bool _visualizingSelectedTags = false;

        public Button StickyCameraButton;


        private List<Button> _mapVisualizationToggleGroup;
        private Color _mapVisualizationColor = Color.white;
        private Color _mapVisualizationSelectedColor = new Color(150 / 255f, 200 / 255f, 150 / 255f);

        [FormerlySerializedAs("simulator")] public SimulationManager simulationManager;
        // Represents a function that modifies the given simulation in some way
        // (for example by changing map visualization mode)
        delegate void SimulationModification(Simulation? simulation);

        private SimulationModification? _mostRecentMapVisualizationModification;

        private void Start() {
            _mapVisualizationToggleGroup = new List<Button>() {
                AllExplorationButton, AllCoverageButton, AllExplorationHeatMapButton, AllCoverageHeatMapButton,
                SelectVisibleAreaButton, SelectedSlamMapButton
            };
            SelectVisualizationButton(AllExplorationButton);
            
            // Set listeners for all map visualization buttons
            AllExplorationButton.onClick.AddListener(() => {
                ExecuteAndRememberMapVisualizationModification((sim) => sim?.ExplorationTracker.ShowAllRobotExploration());
            });
            
            AllCoverageButton.onClick.AddListener(() => {
                ExecuteAndRememberMapVisualizationModification((sim) => sim?.ExplorationTracker.ShowAllRobotCoverage());
            });
            
            AllExplorationHeatMapButton.onClick.AddListener(() => {
                ExecuteAndRememberMapVisualizationModification((sim) => sim?.ExplorationTracker.ShowAllRobotExplorationHeatMap());
            });
            
            AllCoverageHeatMapButton.onClick.AddListener(() => {
                ExecuteAndRememberMapVisualizationModification((sim) => sim?.ExplorationTracker.ShowAllRobotCoverageHeatMap());
            });
            
            SelectVisibleAreaButton.onClick.AddListener(() => {
                ExecuteAndRememberMapVisualizationModification((sim) => {
                    if (sim != null) {
                        if (!sim.HasSelectedRobot()) sim.SelectFirstRobot();
                        sim.ExplorationTracker.ShowSelectedRobotVisibleArea();    
                    }
                });
            });
            
            SelectedSlamMapButton.onClick.AddListener(() => {
                ExecuteAndRememberMapVisualizationModification((sim) => {
                    if (sim != null) {
                        if (!sim.HasSelectedRobot()) sim.SelectFirstRobot();
                        sim.ExplorationTracker.ShowSelectedRobotSlamMap();    
                    }
                });
            });
            
            // Set listeners for Tag visualization buttons 
            AllVisualizeTagsButton.onClick.AddListener(() => {
                ExecuteAndRememberTagVisualization(sim => {
                    if (sim != null) {
                        ToggleVisualizeTagsButtons(AllVisualizeTagsButton);
                    }
                });
            });
            
            VisualizeTagsButton.onClick.AddListener(() => {
                ExecuteAndRememberTagVisualization(sim => {
                    if (sim != null) {
                        if (sim.HasSelectedRobot()) {
                            ToggleVisualizeTagsButtons(VisualizeTagsButton);
                        }
                    }
                });
            });
            
            StickyCameraButton.onClick.AddListener(() => {
                CameraController.singletonInstance.stickyCam = !CameraController.singletonInstance.stickyCam;
                StickyCameraButton.image.color = CameraController.singletonInstance.stickyCam ? _mapVisualizationSelectedColor : _mapVisualizationColor;
            });
        }

        public void Update() {
            if (_visualizingAllTags) {
                simulationManager.CurrentSimulation.ShowAllTags();
            }
            else if (_visualizingSelectedTags) {
                simulationManager.CurrentSimulation.ShowSelectedTags();
            }
            simulationManager.CurrentSimulation.RenderCommunicationLines();
        }

        public void ClearSelectedRobot() {
            _visualizingSelectedTags = false;
            CameraController.singletonInstance.stickyCam = false;
            StickyCameraButton.image.color = _mapVisualizationColor;
            VisualizeTagsButton.image.color = _mapVisualizationColor;
        }

        private void ToggleVisualizeTagsButtons(Button button) {
            simulationManager.CurrentSimulation.ClearVisualTags();
            if (button.name == "AllVisualizeTags") {
                _visualizingSelectedTags = false;
                VisualizeTagsButton.image.color = _mapVisualizationColor;
                _visualizingAllTags = !_visualizingAllTags;
                button.image.color = _visualizingAllTags ? _mapVisualizationSelectedColor : _mapVisualizationColor;
            }
            else {
                _visualizingAllTags = false;
                AllVisualizeTagsButton.image.color = _mapVisualizationColor;
                _visualizingSelectedTags = !_visualizingSelectedTags;
                button.image.color = _visualizingSelectedTags ? _mapVisualizationSelectedColor : _mapVisualizationColor;
            }
        }

        // This function executes the given map visualization change and remembers it.
        // Whenever the simulator creates a new simulation the most recent visualization change is repeated 
        private void ExecuteAndRememberMapVisualizationModification(SimulationModification modificationFunc) {
            _mostRecentMapVisualizationModification = modificationFunc;
            modificationFunc(simulationManager.CurrentSimulation);
        }

        private void ExecuteAndRememberTagVisualization(SimulationModification modificationFunc) {
            modificationFunc(simulationManager.CurrentSimulation);
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

        public void UpdateTagDebugInfo(string info) {
            TagDebugText.text = info;
        }

        // Called whenever the simulator instantiates a new simulation object 
        public void NotifyNewSimulation(Simulation? newSimulation) {
            if (newSimulation != null) {
                newSimulation.ExplorationTracker.OnVisualizationModeChanged += OnMapVisualizationModeChanged;
                _mostRecentMapVisualizationModification?.Invoke(newSimulation);
            }
        }

        private void OnMapVisualizationModeChanged(VisualizationMode mode) {
            if (mode is AllRobotsExplorationVisualization) {
                SelectVisualizationButton(AllExplorationButton);
            } else if (mode is AllRobotsCoverageVisualization) {
                SelectVisualizationButton(AllCoverageButton);
            } else if (mode is ExplorationHeatMapVisualization) {
                SelectVisualizationButton(AllExplorationHeatMapButton);
            } else if (mode is CoverageHeatMapVisualization) {
                SelectVisualizationButton(AllCoverageHeatMapButton);
            } else if (mode is CurrentlyVisibleAreaVisualization) {
                SelectVisualizationButton(SelectVisibleAreaButton);
            } else if (mode is SelectedRobotSlamMapVisualization) {
                SelectVisualizationButton(SelectedSlamMapButton);
            } else {
                throw new Exception($"No registered button matches the Visualization mode {mode.GetType()}");
            }
            
        }
    }
}