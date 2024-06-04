// Copyright 2024 MAES
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
// Contributors: Rasmus Borrisholt Schmidt, Andreas Sebastian Sørensen, Thor Beregaard
// 
// Original repository: https://github.com/Molitany/MAES

using Maes.Map;
using Maes.Utilities;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace Maes.ExplorationAlgorithm.Minotaur
{
    public class Doorway
    {
        public readonly Vector2Int Center;
        public readonly Line2D Opening;
        public IEnumerable<Vector2Int> Tiles => Opening.Rasterize().Select(tile => Vector2Int.FloorToInt(tile));
        public bool Explored;
        public CardinalDirection ExitDirection;
        public static int DoorWidth;
        public static CoarseGrainedMap _map;

        public Doorway(Line2D opening, Vector2Int center, CardinalDirection exitDirection)
        {
            Center = center;
            Explored = false;
            Opening = opening;
            ExitDirection = exitDirection;
        }

        public override bool Equals(object obj)
        {
            if (obj is Doorway other)
            {
                var squareTiles = Enumerable.Range(0, DoorWidth+1).SelectMany(i => Tiles.Select(doorTile => doorTile + ExitDirection.Vector * i)).ToList();
                //squareTiles.ToList().ForEach(tile => _map.FromSlamMapCoordinate(tile).DrawDebugLineFromRobot(_map, Color.cyan));
                if (other.Tiles.Any(tile => squareTiles.Contains(tile)))
                {
                    return true;
                }
            }
            return false;
        }

        public override int GetHashCode()
        {
            return HashCode.Combine(Center, Explored, ExitDirection);
        }
    }
}
