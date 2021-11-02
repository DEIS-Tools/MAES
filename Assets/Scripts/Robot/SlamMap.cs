using System;
using System.Collections.Generic;
using System.Runtime.CompilerServices;
using Dora.MapGeneration;
using Dora.Utilities;
using UnityEditor;
using UnityEditor.UI;
using UnityEngine;
using Random = System.Random;

namespace Dora.Robot {
    public class SlamMap {
        // Size of a tile in world space
        private readonly float _tileSize;
        private readonly int _widthInTiles, _heightInTiles;

        private SlamTileStatus[,] _tiles;
        private SimulationMap<bool> _collisionMap;

        private readonly float _scale;
        private readonly Vector2 _scaledOffset;
        private readonly RobotConstraints _robotConstraints;
        private float _lastInaccuracyX = 0f;
        private float _lastInaccuracyY = 0f;
        // Represents the current approximate position of the given robot
        public Vector2 ApproximatePosition { get; private set; }
        private int _randomSeed;
        private Random random;


        public SlamMap(SimulationMap<bool> collisionMap, RobotConstraints robotConstraints, int randomSeed) {
            _collisionMap = collisionMap;
            _robotConstraints = robotConstraints;
            _randomSeed = randomSeed;
            _widthInTiles = collisionMap.WidthInTiles * 2;
            _heightInTiles = collisionMap.HeightInTiles * 2;
            _scale = collisionMap.Scale;
            _scaledOffset = collisionMap.ScaledOffset;
            _tiles = new SlamTileStatus[_widthInTiles, _heightInTiles];
            this.random = new Random(randomSeed);

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

        

        public enum SlamTileStatus {
            Unseen,
            Open,
            Solid
        }

        public void UpdateApproxPosition(Vector2 worldPosition) {
            var sign = random.Next(2) == 1 ? -1 : 1;
            var multiplier = random.NextDouble() * sign;
            var newInaccuracy = _lastInaccuracyX + multiplier * (_robotConstraints.SlamPositionInaccuracy / 10f);
            newInaccuracy = MathUtilities.Clamp(newInaccuracy, -_robotConstraints.SlamPositionInaccuracy,
                _robotConstraints.SlamPositionInaccuracy);
            var newXAprox = (float)newInaccuracy + worldPosition.x;
            _lastInaccuracyX = (float)newInaccuracy;
            
            sign = random.Next(2) == 1 ? -1 : 1;
            multiplier = random.NextDouble() * sign;
            newInaccuracy = _lastInaccuracyY + multiplier * (_robotConstraints.SlamPositionInaccuracy / 10f);
            newInaccuracy = MathUtilities.Clamp(newInaccuracy, -_robotConstraints.SlamPositionInaccuracy,
                _robotConstraints.SlamPositionInaccuracy);
            var newYAprox = (float)newInaccuracy + worldPosition.y;
            _lastInaccuracyY = (float)newInaccuracy;

            this.ApproximatePosition = new Vector2(newXAprox, newYAprox);
        }
        
        // Synchronizes the given slam maps to create a new one
        public static void Combine(List<SlamMap> maps) {
            var globalMap = new SlamTileStatus[maps[0]._widthInTiles, maps[0]._heightInTiles];
            for (int x = 0; x < globalMap.GetLength(0); x++) 
                for (int y = 0; y < globalMap.GetLength(1); y++) 
                    globalMap[x, y] = SlamTileStatus.Unseen;

            foreach (var map in maps) {
                for (int x = 0; x < map._widthInTiles; x++) {
                    for (int y = 0; y < map._heightInTiles; y++) {
                        if (map._tiles[x, y] != SlamTileStatus.Unseen)
                            globalMap[x, y] = map._tiles[x, y];
                    }
                }
            }

            foreach (var map in maps) {
                map._tiles = globalMap.Clone() as SlamTileStatus[,];
            }
        }
    }
}