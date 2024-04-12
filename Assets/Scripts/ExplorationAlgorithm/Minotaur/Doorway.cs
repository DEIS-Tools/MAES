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
        public readonly IEnumerable<Vector2Int> Tiles;
        public bool Explored;
        public CardinalDirection ApproachedDirection;
        public static int DoorWidth;
        public static CoarseGrainedMap _map;

        public Doorway(Vector2Int start, Vector2Int end, CardinalDirection approachedDirection)
        {
            Center = (start+end)/2;
            Explored = false;
            Opening = new Line2D(start, end);
            Tiles = Opening.Rasterize();
            ApproachedDirection = approachedDirection;
        }

        public override bool Equals(object obj)
        {
            if (obj is Doorway other)
            {
                var squareTiles = Enumerable.Range(1, DoorWidth).SelectMany(i => Tiles.Select(doorTile => doorTile + ApproachedDirection.Vector * i)).ToList();
                squareTiles.ToList().ForEach(tile => tile.DrawDebugLineFromRobot(_map, Color.cyan));
                if (other.Tiles.All(tile => squareTiles.Contains(tile)))
                {
                    Explored = true;
                    return true;
                }
            }
            return false;
        }

        public override int GetHashCode()
        {
            return HashCode.Combine(Center, Explored, ApproachedDirection);
        }
    }
}
