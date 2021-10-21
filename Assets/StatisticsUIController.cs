using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

namespace Dora
{
    public class StatisticsUIController : MonoBehaviour
    {
        
        public Image Mask;
        public Text ProgressPercentageText;

        public void SetExplorationProgress(float progress)
        {
            Mask.fillAmount = progress;
            ProgressPercentageText.text = (progress * 100f).ToString("#.00") + "%";
        }

        public void Update(Simulation currentSimulation)
        {
            SetExplorationProgress(currentSimulation.ExplorationTracker.ExploredProportion);
        }
    }
}
