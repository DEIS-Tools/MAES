using System;
using UnityEngine;

namespace Maes.Utilities {
    public class Geometry {
        public static float DistanceBetween(in Vector2 p1, in Vector2 p2) {
            return Vector2.Distance(p1, p2);
            // if (p1.Equals(p2)) return 0f;
            //
            // var xDelta = p2.x - p1.x;
            // var yDelta = p2.y - p1.y;
            //
            // return Mathf.Sqrt(Mathf.Pow(xDelta, 2f) + Mathf.Pow(yDelta, 2f));
        }
        
        public static Vector2Int FromROSCoord(Vector2Int RosPosition) {
            return new Vector2Int(-RosPosition.x, -RosPosition.y);
        }

        public static Vector2 FromROSCoord(Vector3 RosPosition) {
            return new Vector2(-RosPosition.x, -RosPosition.y);
        }

        public static Vector2 ToROSCoord(Vector3 MapPosition) {
            // Map position is robots position in the tile grid. NOT world position / game object position
            return new Vector2(-MapPosition.x, -MapPosition.y);
        }


        public static Vector2 VectorFromDegreesAndMagnitude(float angleDegrees, float magnitude) {
            var angleRad = angleDegrees * Mathf.Deg2Rad;
            return new Vector2(Mathf.Cos(angleRad), Mathf.Sin(angleRad)) * magnitude;
        }

        public static Vector2 DirectionAsVector(float angleDegrees) {
            return new Vector2(Mathf.Cos(angleDegrees * Mathf.Deg2Rad), Mathf.Sin(angleDegrees * Mathf.Deg2Rad));
        }

        public static int ManhattanDistance(Vector2Int v1, Vector2Int v2) {
            return Math.Abs(v1.x - v2.x) + Math.Abs(v1.y - v2.y);
        }
    }
}