using System.Collections.Generic;
using Maes.Robot;
using Maes.Statistics;

namespace Maes.Map.Visualization {
    public class SelectedRobotSlamMapVisualization : VisualizationMode{
        
        private SimulationMap<ExplorationCell> _explorationMap;
        private Robot2DController _robot;
        private SlamMap _map;

        public SelectedRobotSlamMapVisualization(SimulationMap<ExplorationCell> explorationMap, Robot2DController robot) {
            _explorationMap = explorationMap;
            _robot = robot;
            _map = _robot.SlamMap;
        }

        public void RegisterNewlyExploredCells(MonaRobot robot, IEnumerable<(int, ExplorationCell)> exploredCells) {
            // Ignore since entire map is replaced every tick
        }

        public void RegisterNewlyCoveredCells(MonaRobot robot, IEnumerable<(int, ExplorationCell)> coveredCells) {
            // Ignore since entire map is replaced every tick
        }

        public void UpdateVisualization(ExplorationVisualizer visualizer, int currentTick) {
            visualizer.SetAllColors((index) => {
                var coordinate = _map.TriangleIndexToCoordinate(index);
                var status = _map.GetStatusOfTile(coordinate);

                return status switch {
                    SlamMap.SlamTileStatus.Open => ExplorationVisualizer.SlamSeenColor,
                    SlamMap.SlamTileStatus.Solid => ExplorationVisualizer.SolidColor,
                    _ => ExplorationVisualizer.StandardCellColor
                };
            });
        }
    }
}