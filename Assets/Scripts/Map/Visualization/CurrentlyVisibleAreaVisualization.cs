using System.Collections.Generic;
using Maes.Robot;
using Maes.Statistics;
using UnityEngine;

namespace Maes.Map.Visualization {
    public class CurrentlyVisibleAreaVisualization : VisualizationMode {
        
        private SimulationMap<ExplorationCell> _explorationMap;
        private Robot2DController _selectedRobot;

        public CurrentlyVisibleAreaVisualization(SimulationMap<ExplorationCell> explorationMap, Robot2DController selectedRobot) {
            _selectedRobot = selectedRobot;
            _explorationMap = explorationMap;
        }

        public void RegisterNewlyExploredCells(MonaRobot robot, IEnumerable<(int, ExplorationCell)> exploredCells) {
            /* Ignore new exploration data as we are interested in all VISIBLE cells */
        }

        public void RegisterNewlyCoveredCells(MonaRobot robot, IEnumerable<(int, ExplorationCell)> coveredCells) {
            /* Coverage data not needed */
        }

        public void UpdateVisualization(ExplorationVisualizer visualizer, int currentTick) {
            visualizer.SetAllColors(_explorationMap, ExplorationCellToColor);
        }
        
        private Color32 ExplorationCellToColor(int index) {
            return _selectedRobot.SlamMap._currentlyVisibleTriangles.Contains(index) ? 
                    ExplorationVisualizer.ExploredColor : ExplorationVisualizer.StandardCellColor;
        }
    }
}