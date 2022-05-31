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
using UnityEngine;

namespace Maes.Map.PathFinding {
    // Represents a single line in a path. Used for creating tile reservations when traversing the path
    public class PathStep {
        public readonly Vector2Int Start;
        public readonly Vector2Int End;
        public readonly HashSet<Vector2Int> CrossedTiles;

        public PathStep(Vector2Int start, Vector2Int end, HashSet<Vector2Int> crossedTiles) {
            Start = start;
            End = end;
            CrossedTiles = crossedTiles;
        }
    }
}