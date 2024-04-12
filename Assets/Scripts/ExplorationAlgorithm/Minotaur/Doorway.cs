using Maes.Utilities;
using System;
using System.Collections;
using System.Collections.Generic;
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
            if (obj is Doorway other
                && Center == other.Center
                && Explored == other.Explored
                && ApproachedDirection == other.ApproachedDirection)
            {
                return true;
            }
            return false;
        }

        public override int GetHashCode()
        {
            return HashCode.Combine(Center, Explored, ApproachedDirection);
        }
    }
}
