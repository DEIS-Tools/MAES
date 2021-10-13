using System;
using System.Collections.Generic;
using UnityEngine;

namespace Dora.MapGeneration
{
    public class CollisionMap
    {
        
        public CollisionMap(List<Vector3> vertices, List<int> triangles)
        {
            
        }
        
        private class CollisionMapCell
        {
            // A cell is a rectangle consisting of 8 triangles in 4 different orientations
            // The triangles are indexed and arranged as shown in this very pretty illustration:
            ///  |4/5|6\7|
            ///  |0\1|2/3|
            private bool[] _collisionTriangles;


            
            
            // Converts the given coordinate decimals to the index of the corresponding triangle
            // Example: (x:0.9, y:0.1) -> 4
            private int CoordinateDecimalsToTriangleIndex(float x, float y)
            {
                if (x < 0.0f || x > 1.0f || y < 0.0f || y > 1.0f)
                    throw new ArgumentException("Coordinate decimals must be between 0.0 and 1.0");

                var index = 0;

                if (y < 0.5)
                {
                    // Bottom left quadrant
                    if (x + y < 0.5f) return 0;
                    if (x + y >= 0.5f && x < 0.5f) return 1;
                    // Bottom right quadrant
                    if (x - y < 0.5f) return 2;
                    return 3;
                }
                else
                {
                    // Top left quadrant
                    if (y - x > 0.5f && x < 0.5f) return 4;
                    if (y - x <= 0.5f && x < 0.5f) return 5;
                    // Top right quadrant
                    if (x + y < 1.5f) return 6;
                    return 7;
                }
            }
            
        }
        
    }
}