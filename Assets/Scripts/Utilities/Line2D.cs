using System;
using System.Collections.Generic;
using UnityEngine;

namespace Dora.Utilities
{
    public class Line2D
    {

        public readonly Vector2 Start, End, MidPoint;
        private readonly float _minY, _maxY;
        private readonly float _minX, _maxX;
        
        private readonly bool _isVertical = false;
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
            
            _isVertical = Mathf.Approximately(_minX, _maxX);
            if(!_isVertical) _isHorizontal = Mathf.Approximately(_minY, _maxY);
            
            if(!_isVertical && !_isVertical)
            {
                _a = (end.y - start.y) / (_maxX - _minX);
                _b = start.y - _a * start.x;
            }
        }

        // Checks for intersection with an infinite line described by a_1 x + b_2 
        public Vector2? GetIntersection(float a_2, float b_2)
        {
            if (this._isVertical) // Special case
            {
                // This line is vertical. Simply plug x coordinate of this line into equation to find y intersection
                var yIntersection = this.Start.x * a_2 + b_2;
                // Return intersection if it is within bounds of this line
                if (yIntersection <= _maxY && yIntersection >= _minY)
                    return new Vector2(Start.x, yIntersection);
                else
                    return null;

            }
            else if(this._isHorizontal) // Optimization
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
                var xIntersection = (this.Start.y - b_2) / a_2;
                // Return intersection if it is within bounds of this line
                if (xIntersection >= _minX && xIntersection <= _maxX)
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
                if (intersectX - _minX < -0.0001f || _maxX - intersectX < -0.0001f) return null;
                var intersectY = _a * intersectX + _b;
                // Debug.Log("Line : " + _a + "x + " + _b + " intersects with " + a_2 + "x + " + b_2 + 
                //           " at " + intersectX + ", " + intersectY);
                return new Vector2(intersectX, intersectY);
            }
        }
    }
}