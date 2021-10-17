using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Threading;
using Dora.MapGeneration;
using Dora.Robot;
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
            _explorationVisualizer.SetMap(_explorationMap, collisionMap.Scale, collisionMap.ScaledOffset);
        }
        
        public void Update(SimulationConfiguration config, List<MonaRobot> robots)
        {
            float visibilityRange = 15.0f;
            foreach (var robot in robots)
            {
                for (int i = 0; i < 180; i++)
                {
                    var angle = i * 2;
                    if (i * 2 % 45 == 0) continue;
                    _explorationMap.Raytrace(robot.transform.position, angle, visibilityRange, cell => {
                        cell.IsExplored = true;
                        return cell.isExplorable;
                    });
                }
            }

            _explorationVisualizer.UpdateColors(_explorationMap);
        }

    }
}