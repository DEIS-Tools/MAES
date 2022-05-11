using System;
using System.Collections.Generic;
using Maes.Map;
using UnityEngine;

namespace Maes.Statistics {
    // This class is responsible for calculating which tiles are covered by a robot
    public class CoverageCalculator {

        public delegate void MiniTileConsumer(int index1, ExplorationCell cell1, int index2, ExplorationCell cell2);

        public int CoveredMiniTiles = 0;
        private int CoverableTiles = 0;
        
        private SimulationMap<ExplorationCell> _explorationMap;
        
        public float CoverageProportion => CoveredMiniTiles / (float) CoverableTiles;

        public CoverageCalculator(SimulationMap<ExplorationCell> explorationMap, SimulationMap<bool> collisionMap) {
            _explorationMap = explorationMap;
            FindCoverableTiles(explorationMap, collisionMap);
        }

        private void FindCoverableTiles(SimulationMap<ExplorationCell> explorationMap, SimulationMap<bool> collisionMap) {
            // Register all coverable tiles
            for (int x = 0; x < _explorationMap.WidthInTiles; x++) {
                for (int y = 0; y < _explorationMap.HeightInTiles; y++) {
                    var tileCells = collisionMap.GetTileByLocalCoordinate(x, y).GetTriangles();
                    var explorationCells = _explorationMap.GetTileByLocalCoordinate(x, y).GetTriangles();

                    for (int i = 0; i < 8; i+=2) {
                        // If any of the two triangles forming the 'mini-tile' are solid,
                        // then mark both triangles as non-coverable
                        // (because the entire mini-tile will be considered as solid in the SLAM map used by the robots)
                        var isSolid = tileCells[i] || tileCells[i+1];
                        explorationCells[i].CanBeCovered = !isSolid;
                        explorationCells[i+1].CanBeCovered = !isSolid;
                        if (!isSolid) CoverableTiles++;
                    }
                }
            }
        }
        
        public void UpdateRobotCoverage(Vector2 robotPos, int currentTick, MiniTileConsumer tileConsumer) {
            var robotMiniTileCenterX = (float) Math.Truncate(robotPos.x) + Mathf.Round(robotPos.x - (float) Math.Truncate(robotPos.x)) * 0.5f + ((robotPos.x < 0) ? -0.25f : 0.25f);
            var robotMiniTileCenterY = (float) Math.Truncate(robotPos.y) + Mathf.Round(robotPos.y - (float) Math.Truncate(robotPos.y)) * 0.5f + ((robotPos.y < 0) ? -0.25f : 0.25f);
            var robotMiniTilePos = new Vector2(robotMiniTileCenterX, robotMiniTileCenterY);
            // Loop through alle tiles currently near the robot
            for (int x = -1; x <= 1; x++) {
                for (int y = -1; y <= 1; y++) {
                    var tilePosition = robotMiniTilePos + new Vector2(x * 0.5f, y * 0.5f);
                    // ------------------------------------------------------------------------------------------------
                    // ** The following commented code is bugged - It can be reintroduced and debugged if we need more
                    //    precise coverage tracking. If left commented, all immediate neighbour tiles will be considered
                    //    covered regardless of distance from the robot. **
                    // ------------------------------------------------------------------------------------------------
                    // var centerOffsetX = tilePosition.x < 0 ? -0.25f : 0.25f;
                    // var centerOffsetY = tilePosition.y < 0 ? -0.25f : 0.25f;
                    // // const float centerOffset = 0.25f;
                    // var tileCenterX = Mathf.Floor(tilePosition.x) + centerOffsetX + Mathf.Round(Mathf.Abs(tilePosition.x) % 1.0f) * 0.5f;
                    // var tileCenterY = Mathf.Floor(tilePosition.y) + centerOffsetY + Mathf.Round(Mathf.Abs(tilePosition.y) % 1.0f) * 0.5f;
                    // Debug.Log($"Checking coverage for tile with coordinates ({tileCenterX}, {tileCenterY})");
                    //
                    // // Only consider this tile if they are within coverage range in both x- and y-axis
                    // if (Mathf.Max(Mathf.Abs(tileCenterX - robotPos.x), Mathf.Abs(tileCenterY - robotPos.y)) > centerCoverageRadius && false)
                    //     continue;

                    var (triangle1, triangle2) = 
                    _explorationMap.GetMiniTileTrianglesByWorldCoordinates(tilePosition);

                    // Skip non-coverable tiles
                    if (!triangle1.Item2.CanBeCovered) continue;
                    if (!triangle1.Item2.IsCovered) CoveredMiniTiles++; // Register first coverage of this tile
                    
                    // Execute the given function on the two covered cells
                    tileConsumer(triangle1.Item1, triangle1.Item2, triangle2.Item1,triangle2.Item2);

                    // Register coverage (both repeated and first time) for other statistics such as the heat map 
                    triangle1.Item2.RegisterCoverage(currentTick);
                    triangle2.Item2.RegisterCoverage(currentTick);
                }   
            }
        }
        
    }
}