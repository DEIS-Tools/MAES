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
using System.Linq;
using UnityEngine;

namespace Maes.Utilities
{
    public class Line2D
    {
        public readonly Vector2 Start, End, MidPoint;
        private readonly float _minY, _maxY;
        private readonly float _minX, _maxX;

        public readonly bool IsVertical = false;
        private readonly bool _isHorizontal = false;

        // Describe line by ax + b
        private float _a;
        private float _b;

        public Line2D(Vector2 start, Vector2 end)
        {
            Start = start;
            End = end;
            MidPoint = (start + end) / 2f;

            if (start.x > end.x)
                (start, end) = (end, start);

            _minY = Mathf.Min(start.y, end.y);
            _maxY = Mathf.Max(start.y, end.y);

            _minX = Mathf.Min(start.x, end.x);
            _maxX = Mathf.Max(start.x, end.x);

            IsVertical = Mathf.Approximately(_minX, _maxX);
            if (!IsVertical) _isHorizontal = Mathf.Approximately(_minY, _maxY);

            if (!IsVertical)
            {
                _a = (end.y - start.y) / (_maxX - _minX);
                _b = start.y - _a * start.x;
            }
            else
            {
                _a = (end.x - start.x) / (_maxY - _minY);
                _b = start.x - _a * start.y;
            }
        }

        public List<Vector2> GetPoints()
        {
            return new List<Vector2> { Start, End };
        }

        public float SlopeIntercept(float x)
        {
            return _a * x + _b;
        }


        // Returns true if the y value of the line grows as x increases
        public bool IsGrowing()
        {
            if (IsVertical || _isHorizontal)
                throw new Exception("Cannot call IsGrowing on Horizontal or Vertical lines");
            return _a > 0f;
        }

        public bool SameOrientation(Line2D otherLine)
        {
            return IsVertical && otherLine.IsVertical || _isHorizontal && otherLine._isHorizontal;
        }

        public (Vector2 linePoint, Vector2 otherLinePoint) GetClosestPoints(Line2D otherLine)
        {
            var linePoints = Rasterize();
            var otherLinePoints = otherLine.Rasterize();
            (Vector2 linePoint, Vector2 otherLinePoint) result = (Vector2.zero, Vector2.zero);
            float minDistance = float.MaxValue;
            foreach (var point in linePoints)
            {
                foreach (var otherPoint in otherLinePoints)
                {
                    var distance = Vector2.Distance(point, otherPoint);
                    if (minDistance > distance)
                    {
                        minDistance = distance;
                        result = (point, otherPoint); 
                    }
                }
            }
            return result;
        }

        public Vector2? GetIntersection(Line2D otherline, bool infinite = false)
        {
            if (Mathf.Approximately(_a, otherline._a))
            {
                if (SameOrientation(otherline))
                {
                    if (Mathf.Approximately(_b, otherline._b))
                    {
                        var closestPoints = GetClosestPoints(otherline);
                        return (closestPoints.linePoint + closestPoints.otherLinePoint) / 2;
                    }
                    else
                        return null;
                }
                else
                    return IsVertical ? new(_b, otherline._b) : new(otherline._b, _b);
            }
            else
            {
                if (otherline.IsVertical)
                {
                    return otherline.GetIntersection(_a, _b, infinite);
                }
                return GetIntersection(otherline._a, otherline._b, infinite);
            }
        }

        // Checks for intersection with an infinite line described by a_1 x + b_2 
        public Vector2? GetIntersection(float a_2, float b_2, bool infinite = false)
        {
            if (IsVertical) // Special case
            {
                // This line is vertical. Simply plug x coordinate of this line into equation to find y intersection
                var yIntersection = Start.x * a_2 + b_2;

                // Return intersection if it is within bounds of this line

                if (infinite || yIntersection <= _maxY && yIntersection >= _minY)
                    return new Vector2(Start.x, yIntersection);
                else
                    return null;
            }
            else if (_isHorizontal) // Optimization
            {
                // Parallel lines case
                if (Mathf.Abs(a_2) < 0.0001f)
                {
                    if (Mathf.Abs(b_2 - _b) < 0.0001f)
                        return MidPoint;
                    else
                        return null;
                }

                // y = ax + b, solved for x gives x = (y - b) / a (using the y of this horizontal line)
                var xIntersection = (Start.y - b_2) / a_2;
                // Return intersection if it is within bounds of this line
                if (infinite || xIntersection >= _minX && xIntersection <= _maxX)
                    return new Vector2(xIntersection, Start.y);
                else
                    return null;
            }
            else
            {
                // Parallel lines case
                if (Mathf.Abs(a_2 - _a) <= 0.0001f)
                {
                    // If the two parallel lines intersect then return the midpoint of this line as intersection
                    if (Mathf.Abs(b_2 - _b) < 0.0001f)
                        return MidPoint;
                    else
                        return null;
                }

                // Debug.Log($"({b_2} - {_b}) / ({_a} - {a_2}) ");
                var intersectX = (b_2 - _b) / (_a - a_2);
                // Check if intersection is outside line segment
                if (!infinite && intersectX - _minX < -0.0001f || _maxX - intersectX < -0.0001f) return null;
                var intersectY = _a * intersectX + _b;
                // Debug.Log("Line : " + _a + "x + " + _b + " intersects with " + a_2 + "x + " + b_2 + 
                //           " at " + intersectX + ", " + intersectY);
                return new Vector2(intersectX, intersectY);
            }
        }

        /// <summary>
        /// Check if line contains all points of other line
        /// </summary>
        /// <param name="other"> the other line</param>
        /// <returns>true if line contains all points and false if it does not</returns>
        public bool Contains(Line2D other)
        {
            var points = Rasterize();
            var otherPoints = other.Rasterize();
            return otherPoints.All(point => points.Contains(point));
        }

        /// <summary>
        /// Gets all points on a grid that the line goes through from start to end as integer numbers
        /// </summary>
        /// <param name="granularity">The step size to create points from, defaults to 1 world unit aka. a coarse tile</param>
        /// <returns>List of points</returns>
        public IEnumerable<Vector2> Rasterize(float granularity = 1)
        {
            var start = IsVertical ? Start.y : Start.x;
            var end = IsVertical ? End.y : End.x;
            if (start > end)
            {
                (end, start) = (start, end);
            }

            var points = new List<Vector2>();
            for (var x = start; x <= end; x += granularity)
            {
                points.Add(IsVertical ? new Vector2(SlopeIntercept(x), x) : new Vector2(x, SlopeIntercept(x)));
            }
            return points.Distinct();
        }

        public bool EqualLineSegment(Line2D other)
        {
            return (Start == other.Start && End == other.End) || (Start == other.End && End == other.Start);

        }

        public override bool Equals(object obj)
        {
            if (obj == null) return false;
            var otherLine = obj as Line2D;

            return Mathf.Approximately(_a, otherLine._a) &&
                   Mathf.Approximately(_b, otherLine._b) &&
                   (IsVertical == otherLine.IsVertical);
        }

        public override int GetHashCode()
        {
            return HashCode.Combine(IsVertical, _a, _b);
        }
    }
}
