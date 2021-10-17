using System;
using System.Collections.Generic;
using UnityEngine;

namespace Dora.Utilities
{
    public class AxialLine2D
    {

        public readonly Vector2 Start, End;
        private readonly float _minY, _maxY;
        private readonly float _minX, _maxX;
        private readonly bool _isHorizontal;

        public AxialLine2D(Vector2 start, Vector2 end)
        {
            Start = start;
            End = end;

            _minY = Mathf.Min(start.y, end.y);
            _maxY = Mathf.Max(start.y, end.y);
            
            _minX = Mathf.Min(start.x, end.x);
            _maxX = Mathf.Max(start.x, end.x);


            if (Mathf.Approximately(start.x, end.x))
            {
                _isHorizontal = false;
            } else if (Mathf.Approximately(start.y, end.y))
            {
                _isHorizontal = true;
            }
            else
            {
                throw new ArgumentException("An axial line must be ether horizontal or vertical. " +
                                            "The given line was not: { " + start + ", " + end + "}");
            }

        }

        // Checks for intersection with an infinite line described by ax+b 
        public Vector2? GetIntersection(float a, float b)
        {
            if (this._isHorizontal)
            {
                // Parallel lines case
                if (Mathf.Abs(a) < 0.01f)
                {
                    throw new ArgumentException("Due to floating point imprecision intersection can only be " +
                                                "measured for lines that are not near-constant " +
                                                "(the slope, a, must satisfy: a > 0.01f || a < -0.01f)" +
                                                "+ | Given slope: " + a);
                }
                
                // y = ax + b, solved for x gives x = (y - b) / a
                var xIntersection = (this.Start.y - b) / a;
                // Return intersection if it is within bounds of this line
                if (xIntersection >= _minX && xIntersection <= _maxX)
                    return new Vector2(xIntersection, Start.y);
                else 
                    return null;
            }
            else
            {
                // This line is vertical. Simply plug x coordinate of this line into equation to find y intersection
                var yIntersection = this.Start.x * a + b;
                // Return intersection if it is within bounds of this line
                if (yIntersection <= _maxY && yIntersection >= _minY)
                    return new Vector2(Start.x, yIntersection);
                else
                    return null;

            }
        }
    }
}