using Maes;
using UnityEngine;
using UnityEngine.UI;

namespace Dora
{
    public class StatisticsUIController : MonoBehaviour
    {
        public Image Mask;
        public Text ProgressPercentageText;

        public Text ExplorationRateText;

        public void SetExplorationProgress(float progress)
        {
            Mask.fillAmount = progress;
            ProgressPercentageText.text = (progress * 100f).ToString("#.00") + "%";
        }

        public void UpdateStatistics(Simulation currentSimulation)
        {
            SetExplorationProgress(currentSimulation.ExplorationTracker.ExploredProportion);
            ExplorationRateText.text = "Exploration rate (cells/minute): " +
                                       (currentSimulation.ExplorationTracker.ExploredTriangles /
                                        currentSimulation.SimulateTimeSeconds).ToString("#.0")
                                       ;
        }
    }
}
