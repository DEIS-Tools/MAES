using System;
using System.Collections.Generic;
using System.Linq;
using Dora.ExplorationAlgorithm;
using Dora.Robot;
using Dora.Utilities;
using UnityEngine;
using static Dora.Robot.SlamMap;

namespace Dora.MapGeneration.PathFinding {
    
    // This represents a low-resolution map where the robot can comfortably fit inside a single cell
    public class CoarseGrainedMap : IPathFindingMap {
        
        private SlamMap _slamMap;
        private bool[,] _tilesExploredStatus;
        private HashSet<Vector2Int> _excludedTiles;
        private int _width, _height;
        private Vector2 _offset;
        private AStar _aStar;

        public CoarseGrainedMap(SlamMap slamMap, int width, int height, Vector2 offset) {
            _slamMap = slamMap;
            _width = width;
            _height = height;
            _offset = offset;
            _tilesExploredStatus = new bool[width, height];
            _aStar = new AStar();
        }

        // Returns the approximate position on this map (local tile scale coordinates)
        public Vector2 GetApproximatePosition() {
            return _slamMap.ApproximatePosition - _offset;
        }

        public float GetApproximateGlobalDegrees() {
            return _slamMap.GetRobotAngleDeg();
        }

        // Returns position of the given tile relative to the current position of the robot  
        public RelativePosition GetTileCenterRelativePosition(Vector2Int tileCoord) {
            // Convert to local coordinate
            var robotPosition = GetApproximatePosition();
            var target = new Vector2(tileCoord.x + 0.5f, tileCoord.y + 0.5f);
            var distance = Vector2.Distance(robotPosition, (Vector2) target);
            var angle = Vector2.SignedAngle(Geometry.DirectionAsVector(_slamMap.GetRobotAngleDeg()), target - robotPosition);
            return new RelativePosition(distance, angle);
        }

        // Returns the data stored at the given tile, returning null if no data is present
        public bool IsTileExplored(Vector2Int localCoordinate) {
            AssertWithinBounds(localCoordinate);
            return _tilesExploredStatus[localCoordinate.x, localCoordinate.y];
        }
        
        // Sets the data at the given tile, overwriting any existing data object if present
        public void SetTileExplored(Vector2Int localCoordinate, bool data) {
            AssertWithinBounds(localCoordinate);
            _tilesExploredStatus[localCoordinate.x, localCoordinate.y] = data;
        }

        private void AssertWithinBounds(Vector2Int coordinate) {
            var withinBounds= coordinate.x >= 0 && coordinate.x < _width && coordinate.y >= 0 && coordinate.y < _height;
            if (!withinBounds)
                throw new ArgumentException($"Given coordinate is out of bounds {coordinate} ({_width}, {_height})");
        }

        delegate SlamTileStatus StatusAggregator(SlamTileStatus s1, SlamTileStatus s2); 
        
        // Returns the status of the given tile (Solid, Open or Unseen)
        public SlamTileStatus GetTileStatus(Vector2Int localCoordinate, bool optismistic = false) {
            var slamCoord = ToSlamMapCoordinate(localCoordinate);

            var status = _slamMap.GetStatusOfTile(slamCoord);
            if (optismistic) {
                status = AggregateStatusOptimistic(status, _slamMap.GetStatusOfTile(slamCoord + Vector2Int.right));
                status = AggregateStatusOptimistic(status, _slamMap.GetStatusOfTile(slamCoord + Vector2Int.up));
                status = AggregateStatusOptimistic(status, _slamMap.GetStatusOfTile(slamCoord + Vector2Int.right + Vector2Int.up));    
            } else {
                status = AggregateStatusPessimistic(status, _slamMap.GetStatusOfTile(slamCoord + Vector2Int.right));
                status = AggregateStatusPessimistic(status, _slamMap.GetStatusOfTile(slamCoord + Vector2Int.up));
                status = AggregateStatusPessimistic(status, _slamMap.GetStatusOfTile(slamCoord + Vector2Int.right + Vector2Int.up));
            }
            
            return status;
        }
        
        // Combines two SlamTileStatus in a 'optimistic' fashion.
        // If any status is solid both are consider solid. Otherwise, if any status is open both are considered open
        // Unseen is returned only if all statuses are unseen 
        private SlamTileStatus AggregateStatusOptimistic(SlamTileStatus status1, SlamTileStatus status2) {
            if (status1 == SlamTileStatus.Solid || status2 == SlamTileStatus.Solid)
                return SlamTileStatus.Solid;
            if (status1 == SlamTileStatus.Open || status2 == SlamTileStatus.Open)
                return SlamTileStatus.Open;
            return SlamTileStatus.Unseen;
        }

        // Combines two SlamTileStatus in a 'pessimistic' fashion.
        // If any status is solid both are consider solid. If any status is unseen both are considered unseen 
        private SlamTileStatus AggregateStatusPessimistic(SlamTileStatus status1, SlamTileStatus status2) {
            if (status1 == SlamTileStatus.Solid || status2 == SlamTileStatus.Solid)
                return SlamTileStatus.Solid;
            if (status1 == SlamTileStatus.Unseen || status2 == SlamTileStatus.Unseen)
                return SlamTileStatus.Unseen;
            return SlamTileStatus.Open;
        }
        
        
        // Converts the given Slam map coordinate to a local coordinate
        // The Slam map has twice as many tiles in each direction
        public Vector2Int FromSlamMapCoordinate(Vector2Int slamCoord) {
            return slamCoord / 2;
        }

        // Converts the given 
        public Vector2Int ToSlamMapCoordinate(Vector2Int localCoordinate) {
            return localCoordinate * 2;
        }

        // Returns the neighbour in the given direction relative to the current direction of the robot
        public Vector2Int GetRelativeNeighbour(CardinalDirection.RelativeDirection relativeDirection) {
            CardinalDirection currentCardinalDirection = CardinalDirection.DirectionFromDegrees(_slamMap.GetRobotAngleDeg());
            CardinalDirection targetDirection = currentCardinalDirection.GetRelativeDirection(relativeDirection);

            var currentPosition = GetApproximatePosition();
            var relativePosition = currentPosition + targetDirection.Vector;
            return new Vector2Int((int) relativePosition.x, (int) relativePosition.y);
        }

        // Returns the neighbour in the given cardinal direction (relative to global direction)
        public Vector2Int GetGlobalNeighbour(CardinalDirection direction) {
            var currentPosition = GetApproximatePosition();
            var relativePosition = currentPosition + direction.Vector;
            return new Vector2Int((int) relativePosition.x, (int) relativePosition.y);
        }

        public List<Vector2Int>? GetPath(Vector2Int target) {
            var approxPosition = GetApproximatePosition();
            return _aStar.GetOptimisticPath(new Vector2Int((int) approxPosition.x, (int) approxPosition.y), target, this);
        }
        
        public List<Vector2Int>? GetPath(Vector2Int target, HashSet<Vector2Int> excludedTiles) {
            var approxPosition = GetApproximatePosition();
            _excludedTiles = excludedTiles;
            var path = _aStar.GetOptimisticPath(new Vector2Int((int) approxPosition.x, (int) approxPosition.y), target, this); 
            _excludedTiles = new HashSet<Vector2Int>();
            return path;

        }

        public bool IsSolid(Vector2Int coordinate) {
            if (_excludedTiles.Contains(coordinate))
                return true;
            var tileStatus = GetTileStatus(coordinate, optismistic: false); 
            return tileStatus != SlamTileStatus.Open;
        }

        public bool IsOptimisticSolid(Vector2Int coordinate) {
            if (_excludedTiles.Contains(coordinate))
                return true;
            var tileStatus = GetTileStatus(coordinate, optismistic: true); 
            return tileStatus != SlamTileStatus.Open;
        }

        public float CellSize() {
            return 1.0f; 
        }

        public Vector2Int GetCurrentTile() {
            var robotPosition = GetApproximatePosition();
            return new Vector2Int((int) robotPosition.x, (int) robotPosition.y);
        }

        public static void Synchronize(List<CoarseGrainedMap> maps) {
            var globalMap = new bool[maps[0]._width, maps[0]._width];
            
            foreach (var map in maps) {
                for (int x = 0; x < map._width; x++) {
                    for (int y = 0; y < map._height; y++) {
                        globalMap[x, y] |= map._tilesExploredStatus[x, y];
                    }
                }
            }

            foreach (var map in maps) 
                map._tilesExploredStatus = globalMap.Clone() as bool[,];
        }

        // Returns false only if the tile is known to be solid
        public bool IsPotentiallyExplorable(Vector2Int coordinate) {
            var withinBounds= coordinate.x >= 0 && coordinate.x < _width && coordinate.y >= 0 && coordinate.y < _height;
            // To avoid giving away information which the robot cannot know, tiles outside the map bounds are
            // considered explorable
            if (!withinBounds) return true;

            return (!_tilesExploredStatus[coordinate.x, coordinate.y]) && GetSlamTileStatuses(coordinate).All(status => status != SlamTileStatus.Solid);
        }

        private List<SlamTileStatus> GetSlamTileStatuses(Vector2Int coordinate) {
            var slamCoord = coordinate * 2;
            return new List<SlamTileStatus>() {
                _slamMap.GetStatusOfTile(slamCoord),
                _slamMap.GetStatusOfTile(slamCoord + Vector2Int.right),
                _slamMap.GetStatusOfTile(slamCoord + Vector2Int.up),
                _slamMap.GetStatusOfTile(slamCoord + Vector2Int.up + Vector2Int.right),
            };
        }
    }
    
}