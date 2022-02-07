using UnityEngine;
using UnityEngine.UI;

namespace Maes.UI {
    public class SimulationInfoUIController : MonoBehaviour {
        public Image ExplorationBarMask, CoverageBarMask;
        public Text ProgressPercentageText, CoveragePercentageText;
        public Text ExplorationRateText;

        public Text AlgorithmDebugText;
        public Text ControllerDebugText;

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
    }
}