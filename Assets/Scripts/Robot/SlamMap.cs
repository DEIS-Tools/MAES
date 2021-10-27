using System;
using System.Collections.Generic;
using Dora.MapGeneration;
using UnityEngine;

namespace Dora.Robot
{
    public class SlamMap
    {
        // Size of a tile in world space
        private readonly float _tileSize;
        private readonly int _widthInTiles, _heightInTiles;
        
        private SlamTile[,] _tiles;
        private SimulationMap<bool> _collisionMap;

        public SlamMap(SimulationMap<bool> collisionMap)
        {
            _collisionMap = collisionMap;
            _widthInTiles = collisionMap.WidthInTiles * 2; 
            _heightInTiles = collisionMap.HeightInTiles * 2;
            _tiles = new SlamTile[_widthInTiles, _heightInTiles];
            
            for (int x = 0; x < collisionMap.WidthInTiles; x++)
            {
                for (int y = 0; y < collisionMap.HeightInTiles; y++)
                {
                    int slamY = y * 2;
                    int slamX = x * 2;

                    var collisionTriangles = collisionMap.GetTileByLocalCoordinate(x, y).GetTriangles();
                    _tiles[slamX, slamY] = new SlamTile(collisionTriangles[0] || collisionTriangles[1]);
                    _tiles[slamX + 1, slamY] = new SlamTile(collisionTriangles[2] || collisionTriangles[3]);
                    _tiles[slamX, slamY + 1] = new SlamTile(collisionTriangles[4] || collisionTriangles[5]);
                    _tiles[slamX + 1, slamY + 1] = new SlamTile(collisionTriangles[6] || collisionTriangles[7]);
                }
            }
        }

        public Vector2Int TriangleIndexToCoordinate(int triangleIndex)
        {
            var collisionTileIndex = triangleIndex / 8;
            var localTriangleIndex = triangleIndex % 8;
            var collisionX = collisionTileIndex % _collisionMap.WidthInTiles;
            var collisionY = collisionTileIndex / _collisionMap.WidthInTiles;
            // Y offset is 1 if the triangle is the in upper half of the tile
            var yOffset = localTriangleIndex > 3 ? 1 : 0; 
            // X offset is 1 if the triangle is in the right half of tile
            var xOffset = (localTriangleIndex % 4 > 1) ? 1 : 0;
            return new Vector2Int(collisionX + xOffset, collisionY + yOffset);
        }

        public void SetExploredByTriangle(int triangleIndex)
        {
            var localCoordinate = TriangleIndexToCoordinate(triangleIndex);
            _tiles[localCoordinate.x, localCoordinate.y].Visible = true;
        }

        // Represents the current approximate position of the given robot
        public Vector2 ApproximatePosition { get; private set; }

        // Added the template
        private class SlamTile
        {
            public bool IsSolid;
            public bool Visible = false;

            public SlamTile(bool isSolid)
            {
                IsSolid = isSolid;
            }
        }

        // Synchronizes the given slam maps to create a new one
        public static SlamMap Combine(out List<SlamMap> maps)
        {
            throw new NotImplementedException();
        }
        

    }
}