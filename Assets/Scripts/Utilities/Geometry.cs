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
// Contributors: Rasmus Borrisholt Schmidt, Andreas Sebastian Sørensen, Thor Beregaard, Malte Z. Andreasen, Philip I. Holler and Magnus K. Jensen,
// 
// Original repository: https://github.com/Molitany/MAES

using System;
using System.Collections.Generic;
using System.Linq;
using System.Security.Cryptography.X509Certificates;
using UnityEngine;
using UnityEngine.UIElements;

namespace Maes.Utilities
{
    public class Geometry
    {
        public static float DistanceBetween(in Vector2 p1, in Vector2 p2)
        {
            return Vector2.Distance(p1, p2);
            // if (p1.Equals(p2)) return 0f;
            //
            // var xDelta = p2.x - p1.x;
            // var yDelta = p2.y - p1.y;
            //
            // return Mathf.Sqrt(Mathf.Pow(xDelta, 2f) + Mathf.Pow(yDelta, 2f));
        }

        public static Vector2Int FromROSCoord(Vector2Int RosPosition)
        {
            return new Vector2Int(-RosPosition.x, -RosPosition.y);
        }

        public static Vector2 FromROSCoord(Vector3 RosPosition)
        {
            return new Vector2(-RosPosition.x, -RosPosition.y);
        }

        public static Vector2 ToROSCoord(Vector3 MapPosition)
        {
            // Map position is robots position in the tile grid. NOT world position / game object position
            return new Vector2(-MapPosition.x, -MapPosition.y);
        }

        public static bool IsPointWithinCirle(Vector2Int point, Vector2 circleStartPosition, float maxRadius)
        {
            return Mathf.Pow(point.x - circleStartPosition.x, 2) + Mathf.Pow(point.y - circleStartPosition.y, 2) < Mathf.Pow(maxRadius, 2);
        }

        /// <summary>
        /// Utilizes this inequality (x - <paramref name="circleStartPosition"/>.x)^2 + (y - <paramref name="circleStartPosition"/>.y)^2 = r^2 &lt; <paramref name="radius"/>^2 <para/>
        /// Based on <see href="https://math.stackexchange.com/questions/1307832/how-to-tell-if-x-y-coordinate-is-within-a-circle">How to tell if (X,Y) coordinate is within a Circle</see>
        /// </summary>
        /// <param name="points">The points that gets checked if they are within the circle</param>
        /// <param name="circleStartPosition">The origin point of the circle</param>
        /// <param name="maxRadius">The radius in coarse tiles from the robot. R in the inequality</param>
        /// <returns>Doorway tiles around the robot within range</returns>
        public static IEnumerable<Vector2Int> PointsWithinCircle(IEnumerable<Vector2Int> points, Vector2 circleStartPosition, float maxRadius)
        {
            return points.Where(point => IsPointWithinCirle(point, circleStartPosition, maxRadius));
        }

        public static Vector2 VectorFromDegreesAndMagnitude(float angleDegrees, float magnitude)
        {
            var angleRad = angleDegrees * Mathf.Deg2Rad;
            return new Vector2(Mathf.Cos(angleRad), Mathf.Sin(angleRad)) * magnitude;
        }

        public static Vector2 DirectionAsVector(float angleDegrees)
        {
            return new Vector2(Mathf.Cos(angleDegrees * Mathf.Deg2Rad), Mathf.Sin(angleDegrees * Mathf.Deg2Rad));
        }

        public static int ManhattanDistance(Vector2Int v1, Vector2Int v2)
        {
            return Math.Abs(v1.x - v2.x) + Math.Abs(v1.y - v2.y);
        }

        public static (int minX, int maxX, int minY, int maxY) GetBoundingBox(Vector2Int point, Vector2Int perp, Vector2Int thirdPoint)
        {
            var minX = Mathf.Min(point.x,perp.x,thirdPoint.x);
            var maxX = Mathf.Max(point.x,perp.x,thirdPoint.x);
            var minY = Mathf.Min(thirdPoint.y,perp.y,point.y);
            var maxY = Mathf.Max(perp.y,thirdPoint.y,point.y);
            return (minX, maxX, minY, maxY);
        }
    }
}