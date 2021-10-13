using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace Dora.MapGeneration
{
    public class CollisionMap
    {

        private readonly float _scale;
        private CollisionMapCell[,] _cells;
        
        public CollisionMap(List<Vector3> vertices, List<int> collisionTriangles, int width, int height, Vector3 offset, float mapScale)
        {
            _scale = mapScale;
            _cells = new CollisionMapCell[width, height];
            for (int x = 0; x < width; x++)
            {
                for (int y = 0; y < height; y++)
                {
                    _cells[x, y] = new CollisionMapCell();
                }
            }

            var translatedVertices = (from v in vertices select (v - offset) / _scale).ToList();
            var totalTriangles = collisionTriangles.Count / 3;
            for (int triangleNumber = 0; triangleNumber < totalTriangles; triangleNumber++)
            {
                var index = triangleNumber * 3;
                var v1 = translatedVertices[collisionTriangles[index]];
                var v2 = translatedVertices[collisionTriangles[index + 1]];
                var v3 = translatedVertices[collisionTriangles[index + 2]];
                
                // Find the centroid of the triangle
                var triangleCenter = (v1 + v2 + v3) / 3.0f;
                // Mark the corresponding map triangle as collidable
                _cells[(int) triangleCenter.x, (int) triangleCenter.y]
                    .MarkTriangleAsCollidable(triangleCenter.x % 1.0f, triangleCenter.y % 1.0f);
            }
        }
        
        private class CollisionMapCell
        {
            // A cell is a rectangle consisting of 8 triangles in 4 different orientations
            // The triangles are indexed and arranged as shown in this very pretty illustration:
            ///  |4/5|6\7|
            ///  |0\1|2/3|
            private bool[] _collisionTriangles = {false, false, false, false, false, false, false, false};

            // Converts the given coordinate decimals to the index of the corresponding triangle
            // Example: (x:0.9, y:0.1) -> 4
            
            public void MarkTriangleAsCollidable(float xDecimal, float yDecimal)
            {
                var triangleIndex = CoordinateDecimalsToTriangleIndex(xDecimal, yDecimal);
                _collisionTriangles[triangleIndex] = true;
            }

            public bool HasCollidableTriangle()
            {
                return _collisionTriangles.Any(x => x);
            }
            
            private int CoordinateDecimalsToTriangleIndex(float xDecimal, float yDecimal)
            {
                if (xDecimal < 0.0f || xDecimal > 1.0f || yDecimal < 0.0f || yDecimal > 1.0f)
                    throw new ArgumentException("Coordinate decimals must be between 0.0 and 1.0");

                var index = 0;

                if (yDecimal < 0.5)
                {
                    // Bottom left quadrant
                    if (xDecimal + yDecimal < 0.5f) return 0;
                    if (xDecimal + yDecimal >= 0.5f && xDecimal < 0.5f) return 1;
                    // Bottom right quadrant
                    if (xDecimal - yDecimal < 0.5f) return 2;
                    return 3;
                }
                else
                {
                    // Top left quadrant
                    if (yDecimal - xDecimal > 0.5f && xDecimal < 0.5f) return 4;
                    if (yDecimal - xDecimal <= 0.5f && xDecimal < 0.5f) return 5;
                    // Top right quadrant
                    if (xDecimal + yDecimal < 1.5f) return 6;
                    return 7;
                }
            }
            
        }
        
    }
}