using System;
using System.Collections.Generic;
using Dora.MapGeneration;
using UnityEngine;

namespace Dora.Robot {
    public class SlamMap {
        // Size of a tile in world space
        private readonly float _tileSize;
        private readonly int _widthInTiles, _heightInTiles;

        private SlamTileStatus[,] _tiles;
        private SimulationMap<bool> _collisionMap;

        private readonly float _scale;
        private readonly Vector2 _scaledOffset;

        public SlamMap(SimulationMap<bool> collisionMap) {
            _collisionMap = collisionMap;
            _widthInTiles = collisionMap.WidthInTiles * 2;
            _heightInTiles = collisionMap.HeightInTiles * 2;
            _scale = collisionMap.Scale;
            _scaledOffset = collisionMap.ScaledOffset;
            _tiles = new SlamTileStatus[_widthInTiles, _heightInTiles];

            for (int x = 0; x < _widthInTiles; x++)
            for (int y = 0; y < _heightInTiles; y++)
                _tiles[x, y] = SlamTileStatus.Unseen;
        }

        public Vector2Int TriangleIndexToCoordinate(int triangleIndex) {
            var collisionTileIndex = triangleIndex / 8;
            var localTriangleIndex = triangleIndex % 8;
            var collisionX = collisionTileIndex % _collisionMap.WidthInTiles;
            var collisionY = collisionTileIndex / _collisionMap.WidthInTiles;
            // Y offset is 1 if the triangle is the in upper half of the tile
            var yOffset = localTriangleIndex > 3 ? 1 : 0;
            // X offset is 1 if the triangle is in the right half of tile
            var xOffset = (localTriangleIndex % 4 > 1) ? 1 : 0;
            return new Vector2Int((collisionX * 2) + xOffset, (collisionY * 2) + yOffset);
        }

        public void SetExploredByTriangle(int triangleIndex, bool isOpen) {
            var localCoordinate = TriangleIndexToCoordinate(triangleIndex);
            if (_tiles[localCoordinate.x, localCoordinate.y] != SlamTileStatus.Solid)
                _tiles[localCoordinate.x, localCoordinate.y] = isOpen ? SlamTileStatus.Open : SlamTileStatus.Solid;
        }

        public SlamTileStatus GetTileByTriangleIndex(int triangleIndex) {
            var localCoordinate = TriangleIndexToCoordinate(triangleIndex);
            return _tiles[localCoordinate.x, localCoordinate.y];
        }

        // Represents the current approximate position of the given robot
        public Vector2 ApproximatePosition { get; private set; }

        public enum SlamTileStatus {
            Unseen,
            Open,
            Solid
        }

        // Synchronizes the given slam maps to create a new one
        public static SlamMap Combine(out List<SlamMap> maps) {
            throw new NotImplementedException();
        }
    }
}