using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.CompilerServices;
using Dora.MapGeneration;
using Dora.MapGeneration.PathFinding;
using Dora.Utilities;
using JetBrains.Annotations;
using UnityEditor;
using UnityEngine;
using Random = System.Random;

namespace Dora.Robot {
    public class SlamMap : SlamAlgorithmInterface, IPathFindingMap{
        // Size of a tile in world space
        private readonly float _tileSize;
        private readonly int _widthInTiles, _heightInTiles;
        
        private SlamTileStatus[,] _tiles;
        public Dictionary<Vector2Int, SlamTileStatus> _currentlyVisibleTiles;
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
        
        // Low resolution map
        public CoarseGrainedMap CoarseMap;
        // Low resolution map only considering what is visible now
        public VisibleTilesCoarseMap VisibleTilesCoarseMap;


        public SlamMap(SimulationMap<bool> collisionMap, RobotConstraints robotConstraints, int randomSeed) {
            _collisionMap = collisionMap;
            _robotConstraints = robotConstraints;
            _randomSeed = randomSeed;
            _widthInTiles = collisionMap.WidthInTiles * 2;
            _heightInTiles = collisionMap.HeightInTiles * 2;
            _scale = collisionMap.Scale;
            _scaledOffset = collisionMap.ScaledOffset;
            _tiles = new SlamTileStatus[_widthInTiles, _heightInTiles];

            _currentlyVisibleTiles = new Dictionary<Vector2Int, SlamTileStatus>();
            this.random = new Random(randomSeed);
            _pathFinder = new AStar();
            
            CoarseMap = new CoarseGrainedMap(this, collisionMap.WidthInTiles, collisionMap.HeightInTiles, _scaledOffset);
            VisibleTilesCoarseMap = new VisibleTilesCoarseMap(this, collisionMap.WidthInTiles,
                collisionMap.HeightInTiles, _scaledOffset);

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

        public Vector2Int GetCurrentPositionSlamTile() {
            var currentPosition = this.GetApproxPosition();
            // Since the resolution of the slam map is double, we round to nearest half
            // This is done by multiplying by 2 and then rounding to nearest number.
            // Dividing by two then gives us number with a possible fraction of 0.5
            // We then multiply by 2 again to get to the right slam tile
            var xFloat = Math.Round(currentPosition.x * 2, MidpointRounding.AwayFromZero) / 2;
            var yFloat = Math.Round(currentPosition.y * 2, MidpointRounding.AwayFromZero) / 2;
            var x = Convert.ToInt32(xFloat * 2);
            var y = Convert.ToInt32(yFloat * 2);
            var slamX = (x - ((int)_scaledOffset.x) * 2);
            var slamY = (y - ((int)_scaledOffset.y) * 2);

            return new Vector2Int(slamX, slamY);
        }

        public void ResetRobotVisibility() {
            _currentlyVisibleTiles = new Dictionary<Vector2Int, SlamTileStatus>();
        }

        public void SetCurrentlyVisibleByTriangle(int triangleIndex, bool isOpen) {
            var localCoordinate = TriangleIndexToCoordinate(triangleIndex);

            
            if (!_currentlyVisibleTiles.ContainsKey(localCoordinate)) {
                var newStatus = isOpen ? SlamTileStatus.Open : SlamTileStatus.Solid;
                _currentlyVisibleTiles[localCoordinate] = newStatus;
                CoarseMap.UpdateTile(CoarseMap.FromSlamMapCoordinate(localCoordinate), newStatus);
            }
            else if (_currentlyVisibleTiles[localCoordinate] != SlamTileStatus.Solid) {
                var newStatus = isOpen ? SlamTileStatus.Open : SlamTileStatus.Solid;
                _currentlyVisibleTiles[localCoordinate] = newStatus;
                CoarseMap.UpdateTile(CoarseMap.FromSlamMapCoordinate(localCoordinate), newStatus);
            }
                
        }

        public SlamTileStatus GetVisibleTileByTriangleIndex(int triangleIndex) {
            var localCoordinate = TriangleIndexToCoordinate(triangleIndex);
            return _currentlyVisibleTiles.ContainsKey(localCoordinate) ? _currentlyVisibleTiles[localCoordinate] : SlamTileStatus.Unseen;
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
            if (Math.Abs(_robotConstraints.SlamPositionInaccuracy) < 0.0000001f) {
                this.ApproximatePosition = worldPosition;
                return;
            }
                
            
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
        public static void Synchronize(List<SlamMap> maps) {
            var globalMap = new SlamTileStatus[maps[0]._widthInTiles, maps[0]._heightInTiles];
            for (int x = 0; x < globalMap.GetLength(0); x++) 
                for (int y = 0; y < globalMap.GetLength(1); y++) 
                    globalMap[x, y] = SlamTileStatus.Unseen;
            
            foreach (var map in maps) {
                for (int x = 0; x < map._widthInTiles; x++) {
                    for (int y = 0; y < map._heightInTiles; y++) {
                        if (map._tiles[x, y] != SlamTileStatus.Unseen && globalMap[x, y] != SlamTileStatus.Solid)
                            globalMap[x, y] = map._tiles[x, y];
                    }
                }
            }

            foreach (var map in maps) 
                map._tiles = globalMap.Clone() as SlamTileStatus[,];
            
            // Synchronize coarse maps
            CoarseGrainedMap.Synchronize(maps.Select(m => m.CoarseMap).ToList(), globalMap);
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
            return _currentlyVisibleTiles;
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
            slamCoordinate = new Vector2Int(slamCoordinate.x, slamCoordinate.y);
            
            if (IsWithinBounds(slamCoordinate)) {
                var status = AggregateStatusOptimistic(_tiles[slamCoordinate.x, slamCoordinate.y], _tiles[slamCoordinate.x + 1, slamCoordinate.y]);
                status = AggregateStatusOptimistic(status, _tiles[slamCoordinate.x, slamCoordinate.y + 1]);
                status = AggregateStatusOptimistic(status, _tiles[slamCoordinate.x + 1, slamCoordinate.y + 1]);
                return status == SlamTileStatus.Solid || status == SlamTileStatus.Unseen;
            }
            
            // Tiles outside map bounds are considered solid
            return true;
        }

        // Combines two SlamTileStatus in a 'optimistic' fashion.
        // If any status is solid both are consider solid. Otherwise if any status is open both are considered open 
        private SlamTileStatus AggregateStatusOptimistic(SlamTileStatus status1, SlamTileStatus status2) {
            if (status1 == SlamTileStatus.Solid || status2 == SlamTileStatus.Solid)
                return SlamTileStatus.Solid;
            if (status1 == SlamTileStatus.Open || status2 == SlamTileStatus.Open)
                return SlamTileStatus.Open;
            
            return SlamTileStatus.Unseen;
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
        
        // Returns position of the given tile relative to the current position of the robot  
        public RelativePosition GetRelativeSlamPosition(Vector2Int slamTileTarget) {
            // Convert to local coordinate
            var robotPosition = GetCurrentPositionSlamTile();  
            var distance = Vector2.Distance(robotPosition, (Vector2) slamTileTarget);
            var angle = Vector2.SignedAngle(Geometry.DirectionAsVector(GetRobotAngleDeg()), slamTileTarget - robotPosition);
            return new RelativePosition(distance, angle);
        }

        public RelativePosition GetRelativePosition(Vector2Int target) {
            var robotPosition = GetApproxPosition();
            var distance = Vector2.Distance(robotPosition, (Vector2) target);
            var angle = Vector2.SignedAngle(Geometry.DirectionAsVector(GetRobotAngleDeg()), target - robotPosition);
            return new RelativePosition(distance, angle);
        }

        public float CellSize() {
            return _collisionMap.Scale;
        }
        
        public List<Vector2Int>? GetPath(Vector2Int coarseTileFrom, Vector2Int coarseTileTo, bool acceptPartialPaths = false) {
            var path = _pathFinder.GetPath(coarseTileFrom, coarseTileTo, this, acceptPartialPaths);

            if (path == null)
                return null;

            // Due to rounding errors when converting slam tiles to path tiles, the target may not be correct
            // This replaces the final tile with the actual target.
            path[path.Count - 1] = coarseTileTo;

            return path;
        }

        public List<Vector2Int> GetOptimisticPath(Vector2Int coarseTileFrom, Vector2Int coarseTileTo, bool acceptPartialPaths = false) {
            var path = _pathFinder.GetOptimisticPath(coarseTileFrom, coarseTileTo, this, acceptPartialPaths);

            if (path == null)
                return null;

            // Due to rounding errors when converting slam tiles to path tiles, the target may not be correct
            // This replaces the final tile with the actual target.
            path[path.Count - 1] = coarseTileTo;

            return path;
        }

        public CoarseGrainedMap GetCoarseMap() {
            return CoarseMap;
        }

        public VisibleTilesCoarseMap GetVisibleTilesCoarseMap() {
            return VisibleTilesCoarseMap;
        }
    }
}