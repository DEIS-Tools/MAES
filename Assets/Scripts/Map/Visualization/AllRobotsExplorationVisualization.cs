using System.Collections.Generic;
using Maes.Robot;
using Maes.Statistics;
using UnityEngine;

namespace Maes.Map.Visualization {
    public class AllRobotsExplorationVisualization : VisualizationMode {

        private SimulationMap<ExplorationCell> _explorationMap;
        private HashSet<(int, ExplorationCell)> _newlyExploredCells = new HashSet<(int, ExplorationCell)>();
        private bool _hasBeenInitialized = false;

        public AllRobotsExplorationVisualization(SimulationMap<ExplorationCell> explorationMap) {
            _explorationMap = explorationMap;
        }

        public void RegisterNewlyExploredCells(MonaRobot robot, IEnumerable<(int, ExplorationCell)> exploredCells) {
            foreach (var cellWithIndex in exploredCells)
                _newlyExploredCells.Add(cellWithIndex);
        }

        public void RegisterNewlyCoveredCells(MonaRobot robot, IEnumerable<(int, ExplorationCell)> coveredCells) {
            /* Ignore coverage */
        }

        public void UpdateVisualization(ExplorationVisualizer visualizer, int currentTick) {
            if (_hasBeenInitialized) {
                visualizer.UpdateColors(_newlyExploredCells, ExplorationCellToColor);
                _newlyExploredCells.Clear();
            } else {
                // In the first iteration of this visualizer overwrite all colors of previous visualization mode
                visualizer.SetAllColors(_explorationMap, ExplorationCellToColor);
                _hasBeenInitialized = true;
            }
        }
        
        private static Color32 ExplorationCellToColor(ExplorationCell cell) {
            if (!cell.IsExplorable) return ExplorationVisualizer.SolidColor;
            return (cell.IsExplored) ? ExplorationVisualizer.ExploredColor : ExplorationVisualizer.StandardCellColor;
        }
    }
}