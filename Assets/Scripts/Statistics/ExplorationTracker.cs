using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Threading;
using Dora.MapGeneration;
using UnityEngine;
using Debug = UnityEngine.Debug;

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

            var sw = Stopwatch.StartNew();
            float traceDistance = 20.0f;
            for (int i = 0; i < 90 * 50; i++)
            {
                float angle = (i * 4 + 1f) % 360f;
                _explorationMap.Raytrace(new Vector2(-0.0f, -0.0f), angle, 20.0f, cell =>  cell.IsExplored = true);
            }
            sw.Stop();
            Debug.Log("Millis for 90 raytraces " + sw.ElapsedMilliseconds);

            _explorationVisualizer.SetMap(_explorationMap, collisionMap.Scale, collisionMap.Offset);
        }

    }
}