using System;
using System.Collections.Generic;
using Dora.MapGeneration;

namespace Dora.Statistics
{
    public class ExplorationTracker
    {
        // The low-resolution collision map used to create the smoothed map that robots are navigating 
        private SimulationMap<bool> _collisionMap;
        private ExplorationVisualizer _explorationVisualizer;
        
        private SimulationMap<ExplorationCell> _explorationMap;
        private readonly int _explorationMapWidth;
        private readonly int _explorationMapHeight;

        private const int UpperLeftTriangle = 0;
        private const int LowerRightTriangle = 1;

        public ExplorationTracker(SimulationMap<bool> collisionMap, ExplorationVisualizer explorationVisualizer)
        {
            _collisionMap = collisionMap;
            _explorationVisualizer = explorationVisualizer;
            _explorationMap = collisionMap.FMap(isCellSolid => new ExplorationCell(!isCellSolid));
            _explorationVisualizer.SetMap(_explorationMap, collisionMap.Scale, collisionMap.Offset);
        }

    }
}