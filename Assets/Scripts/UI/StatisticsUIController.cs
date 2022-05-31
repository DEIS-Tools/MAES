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

using UnityEngine;
using UnityEngine.UI;

namespace Maes.UI
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
