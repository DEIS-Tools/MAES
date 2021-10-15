using System;
using System.Collections.Generic;

namespace Dora.Statistics
{
    public class ExplorationTracker
    {
        // The low-resolution collision map used to create the smoothed map that robots are navigating 
        private int[,] _collisionMap;

        private ExplorableCell[,,] _explorationMap;
        private readonly int _explorationMapWidth;
        private readonly int _explorationMapHeight;

        private const int UpperLeftTriangle = 0;
        private const int LowerRightTriangle = 1;

        public ExplorationTracker(int[,] collisionMap)
        {
            _collisionMap = collisionMap;
            _explorationMap = new ExplorableCell[_explorationMapWidth, _explorationMapHeight, 2];
            for (int x = 0; x < _explorationMapWidth; x++)
            {
                for (int y = 0; y < _explorationMapHeight; y++)
                {
                    _explorationMap[x, y, 0] = new ExplorableCell();
                }
            }
        }


        private class ExplorableCell
        {
            public bool IsExplored = false;
            private HashSet<int> _robotsThatVisited = new HashSet<int>(); 
            private int _coverageTimeInTicks = 0;

            public void markAsVisited(int robotID)
            {
                _robotsThatVisited.Add(robotID);
                _coverageTimeInTicks += 1;
            }

        }
        
    }
}