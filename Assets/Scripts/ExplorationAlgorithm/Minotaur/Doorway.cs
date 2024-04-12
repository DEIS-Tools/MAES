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
        public readonly Vector2Int Start;
        public readonly Vector2Int End;
        public bool Explored;
        public CardinalDirection ApproachedDirection;
        public static int DoorWidth;
        public static CoarseGrainedMap _map;

        public Doorway(Vector2Int start, Vector2Int end, CardinalDirection approachedDirection)
        {
            Center = (start+end)/2;
            Explored = false;
            Start = start;
            End = end;
            ApproachedDirection = approachedDirection;
        }

        public override bool Equals(object obj)
        {
            if (obj is Doorway other)
            {
                var doorTiles = new Line2D(Start, End).Rasterize();
                var squareTiles = Enumerable.Range(1, DoorWidth).SelectMany(i => doorTiles.Select(doorTile => doorTile + ApproachedDirection.Vector * i)).ToList();
                squareTiles.ToList().ForEach(tile => tile.DrawDebugLineFromRobot(_map, Color.cyan));
                if (new Line2D(other.Start, other.End).Rasterize().All(tile => squareTiles.Contains(tile)))
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
