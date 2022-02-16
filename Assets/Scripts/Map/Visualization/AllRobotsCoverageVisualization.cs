using System.Collections.Generic;
using Maes.Robot;
using Maes.Statistics;
using UnityEngine;

namespace Maes.Map.Visualization {
    public class AllRobotsCoverageVisualization : VisualizationMode {
        
        private SimulationMap<ExplorationCell> _explorationMap;
        private HashSet<(int, ExplorationCell)> _newlyCoveredCells = new HashSet<(int, ExplorationCell)>();
        private bool _hasBeenInitialized = false;

        public AllRobotsCoverageVisualization(SimulationMap<ExplorationCell> explorationMap) {
            _explorationMap = explorationMap;
        }

        public void RegisterNewlyExploredCells(MonaRobot robot, IEnumerable<(int, ExplorationCell)> exploredCells) {
            /* Ignore exploration */
        }

        public void RegisterNewlyCoveredCells(MonaRobot robot, IEnumerable<(int, ExplorationCell)> coveredCells) {
            foreach (var cellWithIndex in coveredCells) {
                _newlyCoveredCells.Add(cellWithIndex);
            }
        }

        public void UpdateVisualization(ExplorationVisualizer visualizer, int currentTick) {
            if (_hasBeenInitialized) {
                visualizer.UpdateColors(_newlyCoveredCells, ExplorationCellToColor);
                _newlyCoveredCells.Clear();
            } else {
                // In the first iteration of this visualizer overwrite all colors of previous visualization mode
                visualizer.SetAllColors(_explorationMap, ExplorationCellToColor);
                _hasBeenInitialized = true;
            }
        }
        
        private static Color32 ExplorationCellToColor(ExplorationCell cell) {
            if (!cell.CanBeCovered) return ExplorationVisualizer.SolidColor;
            return (cell.IsCovered) ? ExplorationVisualizer.CoveredColor : ExplorationVisualizer.StandardCellColor;
        }
    }
}