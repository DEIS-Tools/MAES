using System.Collections.Generic;

namespace Dora.Statistics
{
    public class ExplorationCell
    {
        public bool isExplorable;
        public bool IsExplored = false;
        private HashSet<int> _robotsThatVisited = new HashSet<int>(); 
        private int _coverageTimeInTicks = 0;

        public ExplorationCell(bool isExplorable)
        {
            this.isExplorable = isExplorable;
        }

        public void markAsVisited(int robotID)
        {
            _robotsThatVisited.Add(robotID);
            _coverageTimeInTicks += 1;
        }
    }
}