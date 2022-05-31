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

namespace Maes.Map.MapGen {
    /*public struct Coord {
        public int x;
        public int y;

        public Coord(int x, int y) {
            this.x = x;
            this.y = y;
        }

        public bool Equals(Coord other) {
            return x == other.x && y == other.y;
        }

        public override bool Equals(object obj) {
            return obj is Coord other && Equals(other);
        }

        public override int GetHashCode() {
            unchecked {
                return (x * 397) ^ y;
            }
        }

        public List<Coord> GetAdjacentCoords() {
            List<Coord> adjacentTiles = new List<Coord>();
            for (int x = this.x - 1; x <= this.x + 1; x++) {
                for (int y = this.y - 1; y <= this.y + 1; y++) {
                    if (x == this.x || y == this.y) {
                        adjacentTiles.Add(new Coord(x, y));
                    }
                }
            }

            return adjacentTiles;
        }

        public bool IsAdjacentTo(Coord other) {
            if ((x + 1 == other.x || x - 1 == other.x) && y == other.y) {
                return true;
            }

            if ((y + 1 == other.y || y - 1 == other.y) && x == other.x) {
                return true;
            }

            return false;
        }

        public int ManhattanDistanceTo(Coord other) {
            return Math.Abs(this.x - other.x) + Math.Abs(this.y - other.y);
        }
    }*/
}