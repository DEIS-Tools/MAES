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

using System.Collections.Generic;
using Maes.Map;
using Maes.Map.PathFinding;
using UnityEngine;

namespace Maes.Robot {
    public interface ISlamAlgorithm {
        
        /// <summary>
        /// The approximate position of the robot contains inaccuracy in both x and y coordinate.
        /// The maximum inaccuracy in each axis is defined the <see cref="RobotConstraints"/> of the simulation.
        /// </summary>
        /// <returns> The approximate position as a Vector2 of the robot measured in slam tiles</returns>
        public Vector2 GetApproxPosition();

        /// <summary>
        /// Used to track all tiles that been explored (intersected by the lidar trace of this robot
        /// (or other robots if Slam map synchronization is enabled in <see cref="RobotConstraints"/>)
        /// </summary>
        /// <returns> a dictionary from a Vector2Int slam map coordinate to a tile status.
        /// If the key is not present in the dictionary the tile is not explored. </returns>
        public Dictionary<Vector2Int, SlamMap.SlamTileStatus> GetExploredTiles();
        
        /// <summary>
        /// The method returns all tiles that can currently be seen by the robot. Vision range is determined by
        /// lidar range in <see cref="RobotConstraints"/> 
        /// </summary>
        /// <returns>a dictionary containing the tile status of all currently visible tiles</returns>
        public Dictionary<Vector2Int, SlamMap.SlamTileStatus> GetCurrentlyVisibleTiles();
        
        /// <returns> The current position of the robot as a slam tile coordinate (rounded down) </returns>
        public Vector2Int GetCurrentPosition();

        /// <summary>
        /// Returns the perceived status of the given tile as a <see cref="SlamMap.SlamTileStatus"/>.
        /// If any of the triangles contained in this slam tile have been seen to be solid, the entire tile is
        /// considered solid. If one triangle is unexplored and one triangle is open, the entire tile is optimistically
        /// assumed to be open.  
        /// </summary>
        /// <param name="tile">The coordinate measured in slam tiles</param>
        /// <returns>The perceived <see cref="SlamMap.SlamTileStatus"/> of the given tile</returns>
        public SlamMap.SlamTileStatus GetTileStatus(Vector2Int tile, bool optimistic = false);
        
        /// <returns>The robots orientation in the slam map measured in degrees relative to the x-axis (counter-clockwise)</returns>
        public float GetRobotAngleDeg();
        
        /// <param name="slamTileFrom"> The starting slam tile of the path</param>
        /// <param name="slamTileTo"> The target slam tile of the path </param>
        /// <param name="acceptPartialPaths"> if true the method will return the path to the closest available tile if a full path is not available</param>
        /// <returns>A path represented as a list of tiles that make up the path</returns>
        public List<Vector2Int> GetPath(Vector2Int slamTileFrom, Vector2Int slamTileTo, bool acceptPartialPaths = false);
        
        /// <param name="slamTileFrom"> The starting slam tile of the path</param>
        /// <param name="slamTileTo"> The target slam tile of the path </param>
        /// <param name="acceptPartialPaths"> if true the method will return the path to the closest available tile if a full path is not available </param>
        /// <returns> An optimistic path (partially unknown tiles are considered to be open) path represented as a list of tiles that make up the path </returns>
        public List<Vector2Int> GetOptimisticPath(Vector2Int coarseTileFrom, Vector2Int coarseTileTo, bool acceptPartialPaths = false);
        
        /// <param name="slamTileTarget"> The slam tile to find the relative position of </param>
        /// <returns> Returns the relative position of the center of the given tiles </returns>
        public RelativePosition GetRelativeSlamPosition(Vector2Int slamTileTarget);

        /// <summary>
        /// The coarse map is a map with half the resolution of the slam map.
        /// </summary>
        /// <returns> A reference to the <see cref="CoarseGrainedMap"/> created from this map</returns>
        public CoarseGrainedMap GetCoarseMap();

        /// <summary>
        /// The current visible coarse map is a map with half the resolution of the slam map containing only the
        /// currently visible tiles.
        /// </summary>
        /// <returns> A reference to the <see cref="VisibleTilesCoarseMap"/> created from the currently visible tiles</returns>
        public VisibleTilesCoarseMap GetVisibleTilesCoarseMap();
    }
}
