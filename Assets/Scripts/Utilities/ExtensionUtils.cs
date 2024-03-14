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
using Codice.Client.BaseCommands;
using Maes.Map;
using Maes.Robot;
using UnityEngine;
using static UnityEditor.FilePathAttribute;

namespace Maes.Utilities {
    public static class ExtensionUtils {
        /// <summary>
        /// Extension method for converting a SLAM tile to a TNF cell
        /// </summary>
        public static (Vector2Int, float) ToTnfCell(this KeyValuePair<Vector2Int, SlamMap.SlamTileStatus> tile) {
            return (tile.Key, tile.Value.ToTnfCellValue());
        }

        public static float ToTnfCellValue(this SlamMap.SlamTileStatus status) {
            return status switch
            {
                SlamMap.SlamTileStatus.Unseen => .5f,
                _ => 0f
            };
        }
        public static void DrawDebugLineFromRobot(this Vector2Int tile, CoarseGrainedMap map)
        {
            var robot = map.CoarseToWorld(map.GetCurrentPositionCoarseTile());
            var point1 = map.CoarseToWorld(tile);
            Debug.DrawLine(robot, point1, Color.magenta);
        }
    }
}