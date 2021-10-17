using System;
using UnityEngine;

namespace Dora.Utilities
{
    public class Geometry
    {

        public static float DistanceBetween(Vector2 p1, Vector2 p2)
        {
            if (p1.Equals(p2)) return 0f;
            
            var xDelta = p2.x - p1.x;
            var yDelta = p2.y - p1.y;
            
            return Mathf.Sqrt(Mathf.Pow(xDelta, 2f) + Mathf.Pow(yDelta, 2f));
        }
        
    }
}