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

        public ExplorationCell(bool isExplorable) {
            IsExplorable = isExplorable;
        }

    }
}