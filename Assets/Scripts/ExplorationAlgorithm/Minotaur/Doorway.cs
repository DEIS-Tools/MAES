using Maes.Utilities;
using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Maes.ExplorationAlgorithm.Minotaur
{
    public class Doorway
    {
        public Vector2Int Position;
        public bool Explored;
        public CardinalDirection ApproachedDirection;

        public Doorway(Vector2Int position, CardinalDirection approachedDirection)
        {
            Position = position;
            Explored = false;
            ApproachedDirection = approachedDirection;
        }

        public static bool Equals(Doorway x, Doorway y)
        {
            if (x.Position.x - y.Position.x < 0.2 && x.Position.y - y.Position.y < 0.2 
            && x.Explored == y.Explored 
            && x.ApproachedDirection == y.ApproachedDirection)
            {
                return true;
            }
            return false;
        }


    }
}
