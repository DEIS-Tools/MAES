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
        public CardinalDirection ApproachedDirection;
        public static int DoorWidth;
        public static CoarseGrainedMap _map;

        public Doorway(Vector2Int start, Vector2Int end, CardinalDirection approachedDirection)
        {
            Center = (start+end)/2;
            Explored = false;
            Opening = new Line2D(start, end);
            ApproachedDirection = approachedDirection;
        }

        public override bool Equals(object obj)
        {
            if (obj is Doorway other)
            {
                var squareTiles = Enumerable.Range(0, DoorWidth+1).SelectMany(i => Tiles.Select(doorTile => doorTile + ApproachedDirection.Vector * i)).ToList();
                squareTiles.ToList().ForEach(tile => _map.FromSlamMapCoordinate(tile).DrawDebugLineFromRobot(_map, Color.cyan));
                if (other.Tiles.Any(tile => squareTiles.Contains(tile)))
                {
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
