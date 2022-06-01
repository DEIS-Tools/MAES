// Copyright 2022 MAES
// 
// This file is part of MAES
// 
// MAES is free software: you can redistribute it and/or modify it under
// the terms of the GNU General Public License as published by the
// Free Software Foundation, either version 3 of the License, or (at your option)
// any later version.
// 
// MAES is distributed in the hope that it will be useful, but WITHOUT
// ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
// or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
// Public License for more details.
// 
// You should have received a copy of the GNU General Public License along
// with MAES. If not, see http://www.gnu.org/licenses/.
// 
// Contributors: Malte Z. Andreasen, Philip I. Holler and Magnus K. Jensen
// 
// Original repository: https://github.com/MalteZA/MAES

using System;
using System.Collections.Generic;
using Maes.Robot;
using Maes.Statistics;
using UnityEngine;

namespace Maes.Map.Visualization {
    internal class ExplorationHeatMapVisualization : VisualizationMode {

        private SimulationMap<ExplorationCell> _explorationMap;
        private int _logicTicksBeforeCold = GlobalSettings.TicksBeforeExplorationHeatMapCold;

        public ExplorationHeatMapVisualization(SimulationMap<ExplorationCell> explorationMap) {
            _explorationMap = explorationMap;
        }

        public void RegisterNewlyExploredCells(MonaRobot robot, IEnumerable<(int, ExplorationCell)> exploredCells) {
            /* No need for newly explored cells as the entire map is replaced every tick */
        }

        public void RegisterNewlyCoveredCells(MonaRobot robot, IEnumerable<(int, ExplorationCell)> coveredCells) {
            /* Ignore coverage data */
        }

        public void UpdateVisualization(ExplorationVisualizer visualizer, int currentTick) {
            // The entire map has to be replaced every tick since all colors are time dependent
            visualizer.SetAllColors(_explorationMap, (cell) => ExplorationCellToColor(cell, currentTick));
        }
        
        private Color32 ExplorationCellToColor(ExplorationCell cell, int currentTick) {
            if (!cell.IsExplorable) return ExplorationVisualizer.SolidColor;
            if (!cell.IsExplored) return ExplorationVisualizer.StandardCellColor;
            
            // The color of every single cell is updated every tick (this is very slow on larger maps)
            // If needed this could possibly be optimized to only update the entire map every 10 ticks
            // and only update the currently visible cells the other 9 ticks.
            // (This would require that this class had access to all currently visible cells (not just newly explored cells))   
            var ticksSinceLastExplored = currentTick - cell.LastExplorationTimeInTicks;
            float coldness = Mathf.Min((float) ticksSinceLastExplored / (float) _logicTicksBeforeCold, 1.0f);
            return Color32.Lerp(ExplorationVisualizer.WarmColor, ExplorationVisualizer.ColdColor, coldness);
        }
    }
}