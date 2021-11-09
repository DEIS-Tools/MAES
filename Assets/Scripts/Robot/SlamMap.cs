using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.CompilerServices;
using Dora.MapGeneration;
using Dora.Utilities;
using JetBrains.Annotations;
using UnityEditor;
using UnityEditor.UI;
using UnityEngine;
using Random = System.Random;

namespace Dora.Robot {
    public class SlamMap : SlamAlgorithmInterface, IPathFindingMap{
        // Size of a tile in world space
        private readonly float _tileSize;
        private readonly int _widthInTiles, _heightInTiles;
        
        private SlamTileStatus[,] _tiles;
        private SlamTileStatus[,] _currentlyVisibleTiles;
        private SimulationMap<bool> _collisionMap;
        private IPathFinder _pathFinder;

        private readonly float _scale;
        private readonly Vector2 _scaledOffset;
        private readonly RobotConstraints _robotConstraints;
        private float _lastInaccuracyX = 0f;
        private float _lastInaccuracyY = 0f;
        // Represents the current approximate position of the given robot
        public Vector2 ApproximatePosition { get; private set; }
        private float _robotAngle = 0;
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
            _currentlyVisibleTiles = new SlamTileStatus[_widthInTiles, _heightInTiles];
            this.random = new Random(randomSeed);
            _pathFinder = new AStar();

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

        public Vector2Int LocalCoordinateToPathFindingCoordinate(Vector2Int localCoordinate) {
            return  localCoordinate / 2;
        }

        public void SetExploredByTriangle(int triangleIndex, bool isOpen) {
            var localCoordinate = TriangleIndexToCoordinate(triangleIndex);
            if (_tiles[localCoordinate.x, localCoordinate.y] != SlamTileStatus.Solid)
                _tiles[localCoordinate.x, localCoordinate.y] = isOpen ? SlamTileStatus.Open : SlamTileStatus.Solid;
        }

        public Vector2Int GetCurrentPositionTile() {
            var currentPosition = this.GetApproxPosition();
            var x = Convert.ToInt32(currentPosition.x);
            var y = Convert.ToInt32(currentPosition.y);
            var slamX = (x - (int)_scaledOffset.x) * 2;
            var slamY = (y - (int)_scaledOffset.y) * 2;

            return new Vector2Int(slamX, slamY);
        }

        public void ResetRobotVisibility() {
            _currentlyVisibleTiles = new SlamTileStatus[_widthInTiles, _heightInTiles];
            for (int x = 0; x < _currentlyVisibleTiles.GetLength(0); x++) {
                for (int y = 0; y < _currentlyVisibleTiles.GetLength(1); y++) {
                    _currentlyVisibleTiles[x, y] = SlamTileStatus.Unseen;
                }
            }
        }

        public void SetCurrentlyVisibleByTriangle(int triangleIndex, bool isOpen) {
            var localCoordinate = TriangleIndexToCoordinate(triangleIndex);
            if (_currentlyVisibleTiles[localCoordinate.x, localCoordinate.y] != SlamTileStatus.Solid)
                _currentlyVisibleTiles[localCoordinate.x, localCoordinate.y] = isOpen ? SlamTileStatus.Open : SlamTileStatus.Solid;
        }

        public SlamTileStatus GetVisibleTileByTriangleIndex(int triangleIndex) {
            var localCoordinate = TriangleIndexToCoordinate(triangleIndex);
            return _currentlyVisibleTiles[localCoordinate.x, localCoordinate.y];
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

        public Vector2 GetApproxPosition() {
            return ApproximatePosition;
        }

       public Dictionary<Vector2Int, SlamTileStatus> GetExploredTiles() {
            var res = new Dictionary<Vector2Int, SlamTileStatus>();

            for (int x = 0; x < _widthInTiles; x++) {
                for (int y = 0; y < _heightInTiles; y++) {
                    if (_tiles[x, y] != SlamTileStatus.Unseen)
                        res[new Vector2Int(x, y)] = _tiles[x, y];
                }
            }

            return res;
        }

        public Dictionary<Vector2Int, SlamTileStatus> GetCurrentlyVisibleTiles() {
            var res = new Dictionary<Vector2Int, SlamTileStatus>();
            
            for (int x = 0; x < _widthInTiles; x++) {
                for (int y = 0; y < _heightInTiles; y++) {
                    if (_currentlyVisibleTiles[x, y] != SlamTileStatus.Unseen)
                        res[new Vector2Int(x, y)] = _currentlyVisibleTiles[x, y];
                }
            }

            return res;
        }

        public SlamTileStatus GetStatusOfTile(Vector2Int tile) {
            return _tiles[tile.x, tile.y];
        }

        public float GetRobotAngleDeg() {
            return _robotAngle;
        }

        public void SetApproxRobotAngle(float robotAngle) {
            _robotAngle = robotAngle;
        }

        private bool IsWithinBounds(Vector2Int slamCoordinate) {
            return slamCoordinate.x > 0 && slamCoordinate.x < _widthInTiles &&
                   slamCoordinate.y > 0 && slamCoordinate.y < _heightInTiles;
        }

        public bool IsOptimisticSolid(Vector2Int coordinate) {
            var slamCoordinate = coordinate * 2;
            
            if (IsWithinBounds(slamCoordinate)) {
                var isTraversable = _tiles[slamCoordinate.x, slamCoordinate.y] == SlamTileStatus.Open
                                    || _tiles[slamCoordinate.x, slamCoordinate.y] == SlamTileStatus.Unseen;
                isTraversable &= _tiles[slamCoordinate.x + 1, slamCoordinate.y] == SlamTileStatus.Open
                                 || _tiles[slamCoordinate.x + 1, slamCoordinate.y] == SlamTileStatus.Unseen;
                isTraversable &= _tiles[slamCoordinate.x, slamCoordinate.y + 1] == SlamTileStatus.Open
                                || _tiles[slamCoordinate.x, slamCoordinate.y + 1] == SlamTileStatus.Unseen;
                isTraversable &= _tiles[slamCoordinate.x + 1, slamCoordinate.y + 1] == SlamTileStatus.Open
                                 || _tiles[slamCoordinate.x + 1, slamCoordinate.y + 1] == SlamTileStatus.Unseen;
                return !isTraversable;
            }
            
            // Tiles outside map bounds are considered solid
            return true;
        }

        public bool IsSolid(Vector2Int coordinate) {
            var slamCoordinate = coordinate * 2;
            
            if (IsWithinBounds(slamCoordinate)) {
                var isTraversable = _tiles[slamCoordinate.x, slamCoordinate.y] == SlamTileStatus.Open;
                isTraversable &= _tiles[slamCoordinate.x + 1, slamCoordinate.y] == SlamTileStatus.Open;
                isTraversable &= _tiles[slamCoordinate.x, slamCoordinate.y + 1] == SlamTileStatus.Open;
                isTraversable &= _tiles[slamCoordinate.x + 1, slamCoordinate.y + 1] == SlamTileStatus.Open;
                return !isTraversable;
            }
            
            // Tiles outside map bounds are considered solid
            return true;
        }

        public float CellSize() {
            return _collisionMap.Scale;
        }
        
        public List<Vector2Int>? GetPath(Vector2Int slamTileFrom, Vector2Int slamTileTo) {
            var path = _pathFinder.GetPath(slamTileFrom / 2, slamTileTo / 2, this)
                ?.Select(coord => coord * 2).ToList();

            if (path == null)
                return null;

            // Due to rounding errors when converting slam tiles to path tiles, the target may not be correct
            // This replaces the final tile with the actual target.
            path[path.Count - 1] = slamTileTo;

            return path;
        }

        public List<Vector2Int> GetOptimisticPath(Vector2Int slamTileFrom, Vector2Int slamTileTo) {
            var path = _pathFinder.GetOptimisticPath(slamTileFrom / 2, slamTileTo / 2, this)
                ?.Select(coord => coord * 2).ToList();

            
            if (path == null)
                return null;

            // Due to rounding errors when converting slam tiles to path tiles, the target may not be correct
            // This replaces the final tile with the actual target.
            path[path.Count - 1] = slamTileTo;

            return path;
        }
    }
}