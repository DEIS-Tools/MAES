using System.Collections;
using System.Collections.Generic;
using Maes.Robot;
using Maes.Statistics;

namespace Maes.Map.Visualization {
    public interface VisualizationMode {
        
        public void RegisterNewlyExploredCells(MonaRobot robot, IEnumerable<(int, ExplorationCell)> exploredCells);
        public void RegisterNewlyCoveredCells(MonaRobot robot, IEnumerable<(int, ExplorationCell)> coveredCells);
        public void UpdateVisualization(ExplorationVisualizer visualizer);

    }
}