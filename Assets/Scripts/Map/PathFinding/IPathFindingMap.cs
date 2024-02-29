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

using UnityEngine;
using static Maes.Map.SlamMap;

namespace Maes.Map.PathFinding
{
    public interface IPathFindingMap
    {

        public bool IsSolid(Vector2Int coordinate);

        public bool IsOptimisticSolid(Vector2Int coordinate);

        public bool IsOffsetSolid(Vector2Int nextCoordinate, Vector2Int currentCoordinate);

        public float CellSize();

        public bool IsWithinBounds(Vector2Int coordinate);

        public SlamTileStatus GetTileStatus(Vector2Int coordinate, bool optimistic = false);

        public Vector2Int? GetNearestTileFloodFill(Vector2Int targetCoordinate, SlamTileStatus lookupStatus);
    }
}
