using System;
using System.Collections.Generic;
using System.Linq;
using Dora.Utilities;
using UnityEngine;

namespace Dora.MapGeneration
{
    // A SimulationMap represents a map of square tiles, where each tile is divided into 8 triangles
    public class SimulationMap<TCell>
    {
        private readonly int _width, _height;
        private readonly SimulationMapTile<TCell>[,] _tiles;
        
        public SimulationMap(Functional.Factory<TCell> cellFactory, int width, int height)
        {
            _width = width;
            _height = height;
            _tiles = new SimulationMapTile<TCell>[width, height];
            for (int x = 0; x < width; x++)
            {
                for (int y = 0; y < height; y++)
                {
                    _tiles[x,y] = new SimulationMapTile<TCell>(cellFactory);
                }
            }
        }

        // Private constructor for a pre-specified set of tiles. This is used in the fmap function
        private SimulationMap(SimulationMapTile<TCell>[,] tiles)
        {
            _tiles = tiles;
            _width = tiles.GetLength(0);
            _height = tiles.GetLength(1);
        }

        // Generates a new SimulationMap<T2> by mapping the given function over all cells
        public SimulationMap<TNewCell> FMap<TNewCell>(Func<TCell, TNewCell> mapper)
        {
            // TODO: Magnus - Define the mapping function for the SimulationMap and SimulationMapTile
            SimulationMapTile<TNewCell>[,] mappedTiles = new SimulationMapTile<TNewCell>[_width, _height];
            for (int x = 0; x < _width; x++)
            {
                for (int y = 0; y < _height; y++)
                {
                    mappedTiles[x, y] = _tiles[x, y].FMap(mapper);
                }
            }

            return new SimulationMap<TNewCell>(mappedTiles);
        }
        
        /*public SimulationMap(List<Vector3> vertices, List<int> collisionTriangles, int width, int height, Vector3 offset, float mapScale)
        {
            _scale = mapScale;
            _cells = new SimulationMapCell[width, height];
            for (int x = 0; x < width; x++)
            {
                for (int y = 0; y < height; y++)
                {
                    _cells[x, y] = new SimulationMapCell();
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
        }*/

        
        
    }
}