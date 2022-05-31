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

namespace Maes.Statistics {
    public class ExplorationCell {
        
        // --- Exploration over time --- 
        public readonly bool IsExplorable; 
        public bool IsExplored = false; 

        // --- Coverage over time  ---
        public bool IsCovered = false;
        public bool CanBeCovered = true;
        private HashSet<int> _robotsThatVisited = new HashSet<int>(); 
        
        // --- Redundant Coverage ---
        public int CoverageTimeInTicks = 0; // The amount of ticks that this cell has been covered
        public int ExplorationTimeInTicks = 0; // The amount of ticks that this has been explored
        
        //  --- Heatmap ---
        public int LastExplorationTimeInTicks = 0; // The last time that this cell was seen by a robot 
        public int LastCoverageTimeInTicks = 0; // The last time that this cell was covered by a robot 


        /// <summary>
        /// Called to register that a robot has seen this tile this tick
        /// </summary>
        public void RegisterExploration(int currentTimeTicks) {
            if (!IsExplorable)
                throw new Exception("Registered exploration for a tile that was marked not explorable");
            ExplorationTimeInTicks += 1;
            LastExplorationTimeInTicks = currentTimeTicks;
            IsExplored = true;
        }

        public void RegisterCoverage(int currenTimeTicks) {
            if (!CanBeCovered)
                throw new Exception("Registered coverage for a tile that was marked not coverable");
            CoverageTimeInTicks += 1;
            LastCoverageTimeInTicks = currenTimeTicks;
            IsCovered = true;
        }

        public ExplorationCell(bool isExplorable) {
            IsExplorable = isExplorable;
        }

    }
}