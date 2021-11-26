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