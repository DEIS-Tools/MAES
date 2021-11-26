using System;
using System.Collections.Generic;
using System.Linq;
using Maes.Robot;
using Maes.Utilities;
using UnityEngine;

namespace Maes.Map.PathFinding {

    // This represents a low-resolution map where the robot can comfortably fit inside a single cell
    public class CoarseGrainedMap : IPathFindingMap {

        private readonly SlamMap _slamMap;
        private bool[,] _tilesCoveredStatus;
        private SlamMap.SlamTileStatus[,] _optimisticTileStatuses;
        private HashSet<Vector2Int> _excludedTiles = new HashSet<Vector2Int>();
        private int _width, _height;
        private Vector2 _offset;
        private AStar _aStar;

        public CoarseGrainedMap(SlamMap slamMap, int width, int height, Vector2 offset) {
            _slamMap = slamMap;
            _width = width;
            _height = height;
            _offset = offset;
            _tilesCoveredStatus = new bool[width, height];
            _optimisticTileStatuses = new SlamMap.SlamTileStatus[width, height];
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

        public Dictionary<Vector2Int, SlamMap.SlamTileStatus> GetExploredTiles() {
            var res = new Dictionary<Vector2Int, SlamMap.SlamTileStatus>();

            for (int x = 0; x < _width; x++) {
                for (int y = 0; y < _height; y++) {
                    var pos = new Vector2Int(x, y);
                    if (GetTileStatus(pos) != SlamMap.SlamTileStatus.Unseen)
                        res[pos] = GetTileStatus(pos);
                }
            }

            return res;
        }

        // Returns the data stored at the given tile, returning null if no data is present
        public bool IsTileExplored(Vector2Int localCoordinate) {
            AssertWithinBounds(localCoordinate);
            return _tilesCoveredStatus[localCoordinate.x, localCoordinate.y];
        }

        // Sets the data at the given tile, overwriting any existing data object if present
        public void SetTileExplored(Vector2Int localCoordinate, bool data) {
            AssertWithinBounds(localCoordinate);
            _tilesCoveredStatus[localCoordinate.x, localCoordinate.y] = data;
        }

        private void AssertWithinBounds(Vector2Int coordinate) {
            var withinBounds = coordinate.x >= 0 && coordinate.x < _width && coordinate.y >= 0 && coordinate.y < _height;
            if (!withinBounds)
                throw new ArgumentException($"Given coordinate is out of bounds {coordinate} ({_width}, {_height})");
        }

        delegate SlamMap.SlamTileStatus StatusAggregator(SlamMap.SlamTileStatus s1, SlamMap.SlamTileStatus s2);

        // Returns the status of the given tile (Solid, Open or Unseen)
        public SlamMap.SlamTileStatus GetTileStatus(Vector2Int localCoordinate, bool optismistic = false) {
            var slamCoord = ToSlamMapCoordinate(localCoordinate);

            var status = _slamMap.GetStatusOfTile(slamCoord);
            if (optismistic) {
                status = AggregateStatusOptimistic(status, _slamMap.GetStatusOfTile(slamCoord + Vector2Int.right));
                status = AggregateStatusOptimistic(status, _slamMap.GetStatusOfTile(slamCoord + Vector2Int.up));
                status = AggregateStatusOptimistic(status, _slamMap.GetStatusOfTile(slamCoord + Vector2Int.right + Vector2Int.up));
            }
            else {
                status = AggregateStatusPessimistic(status, _slamMap.GetStatusOfTile(slamCoord + Vector2Int.right));
                status = AggregateStatusPessimistic(status, _slamMap.GetStatusOfTile(slamCoord + Vector2Int.up));
                status = AggregateStatusPessimistic(status, _slamMap.GetStatusOfTile(slamCoord + Vector2Int.right + Vector2Int.up));
            }

            return status;
        }

        // Combines two SlamTileStatus in a 'optimistic' fashion.
        // If any status is solid both are consider solid. Otherwise, if any status is open both are considered open
        // Unseen is returned only if all statuses are unseen 
        private static SlamMap.SlamTileStatus AggregateStatusOptimistic(SlamMap.SlamTileStatus status1, SlamMap.SlamTileStatus status2) {
            if (status1 == SlamMap.SlamTileStatus.Solid || status2 == SlamMap.SlamTileStatus.Solid)
                return SlamMap.SlamTileStatus.Solid;
            if (status1 == SlamMap.SlamTileStatus.Open || status2 == SlamMap.SlamTileStatus.Open)
                return SlamMap.SlamTileStatus.Open;
            return SlamMap.SlamTileStatus.Unseen;
        }

        // Combines two SlamTileStatus in a 'pessimistic' fashion.
        // If any status is solid both are consider solid. If any status is unseen both are considered unseen 
        private SlamMap.SlamTileStatus AggregateStatusPessimistic(SlamMap.SlamTileStatus status1, SlamMap.SlamTileStatus status2) {
            if (status1 == SlamMap.SlamTileStatus.Solid || status2 == SlamMap.SlamTileStatus.Solid)
                return SlamMap.SlamTileStatus.Solid;
            if (status1 == SlamMap.SlamTileStatus.Unseen || status2 == SlamMap.SlamTileStatus.Unseen)
                return SlamMap.SlamTileStatus.Unseen;
            return SlamMap.SlamTileStatus.Open;
        }


        // Converts the given Slam map coordinate to a local coordinate
        // The Slam map has twice as many tiles in each direction
        public Vector2Int FromSlamMapCoordinate(Vector2Int slamCoord) {
            return slamCoord / 2;
        }

        public List<Vector2Int> FromSlamMapCoordinates(List<Vector2Int> slamCoords) {
            var coarseCoords = new HashSet<Vector2Int>();
            foreach (var slamCoord in slamCoords) {
                coarseCoords.Add(FromSlamMapCoordinate(slamCoord));
            }

            return coarseCoords.ToList();
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

        public List<Vector2Int>? GetPath(Vector2Int target, bool acceptPartialPaths = false, bool beOptimistic = true) {
            var approxPosition = GetApproximatePosition();
            return beOptimistic 
                ? _aStar.GetOptimisticPath(new Vector2Int((int) approxPosition.x, (int) approxPosition.y), target, this) 
                : _aStar.GetPath(Vector2Int.RoundToInt(approxPosition), target, this);
        }
        
        public List<Vector2Int>? GetPath(Vector2Int target, HashSet<Vector2Int> excludedTiles = null, float maxPathCost = float.MaxValue) {
            if (excludedTiles != null && excludedTiles.Contains(target))
                return null;

            var approxPosition = GetApproximatePosition();
            if (excludedTiles != null) _excludedTiles = excludedTiles;
            var path = _aStar.GetOptimisticPath(new Vector2Int((int) approxPosition.x, (int) approxPosition.y), target, this, false); 
            _excludedTiles = new HashSet<Vector2Int>();
            return path;
        }

        public List<PathStep>? GetPathSteps(Vector2Int target, HashSet<Vector2Int> excludedTiles = null) {
            if (excludedTiles != null && excludedTiles.Contains(target))
                return null;

            var approxPosition = GetApproximatePosition();
            if (excludedTiles != null) _excludedTiles = excludedTiles;
            var path = _aStar.GetOptimisticPath(new Vector2Int((int) approxPosition.x, (int) approxPosition.y), target, this);
            _excludedTiles = new HashSet<Vector2Int>();
            return path == null ? null : _aStar.PathToSteps(path, 0.4f);
        }
        
        public List<PathStep>? GetTnfPathAsPathSteps(Vector2Int target) {
            var path = GetPath(target, beOptimistic: false);
            return path == null
                ? null
                : _aStar.PathToSteps(path, 0f);
        }

        public bool IsSolid(Vector2Int coordinate) {
            if (_excludedTiles.Contains(coordinate))
                return true;
            var tileStatus = GetTileStatus(coordinate, optismistic: false);
            return tileStatus != SlamMap.SlamTileStatus.Open;
        }

        public bool IsOptimisticSolid(Vector2Int coordinate) {
            if (_excludedTiles.Contains(coordinate))
                return true;
            return _optimisticTileStatuses[coordinate.x, coordinate.y] != SlamMap.SlamTileStatus.Open;
        }

        public float CellSize() {
            return 1.0f;
        }

        public Vector2Int GetCurrentTile() {
            var robotPosition = GetApproximatePosition();
            return new Vector2Int((int) robotPosition.x, (int) robotPosition.y);
        }

        public static void Synchronize(List<CoarseGrainedMap> maps, SlamMap.SlamTileStatus[,] newSlamStatuses) {
            // Synchronize exploration bool statuses
            var globalExplorationStatuses = new bool[maps[0]._width, maps[0]._height];
            foreach (var map in maps) {
                for (int x = 0; x < map._width; x++) {
                    for (int y = 0; y < map._height; y++) {
                        globalExplorationStatuses[x, y] |= map._tilesCoveredStatus[x, y];
                    }
                }
            }
            foreach (var map in maps)
                map._tilesCoveredStatus = globalExplorationStatuses.Clone() as bool[,];

            // Synchronize tile statuses
            var globalMap = new SlamMap.SlamTileStatus[maps[0]._width, maps[0]._height];
            for (int x = 0; x < maps[0]._width; x++) {
                for (int y = 0; y < maps[0]._height; y++) {
                    int slamX = x * 2;
                    int slamY = y * 2;
                    var status = newSlamStatuses[slamX, slamY];
                    status = AggregateStatusOptimistic(status, newSlamStatuses[slamX + 1, slamY]);
                    status = AggregateStatusOptimistic(status, newSlamStatuses[slamX + 1, slamY + 1]);
                    status = AggregateStatusOptimistic(status, newSlamStatuses[slamX, slamY + 1]);
                    globalMap[x, y] = status;
                }
            }
            foreach (var map in maps)
                map._optimisticTileStatuses = globalMap.Clone() as SlamMap.SlamTileStatus[,];
        }

        public static void Combine(CoarseGrainedMap map, List<CoarseGrainedMap> others, SlamMap.SlamTileStatus[,] newSlamStatuses) {
            var globalExplorationStatuses = new bool[map._width, map._height];
            foreach (var other in others) {
                for (int x = 0; x < other._width; x++) {
                    for (int y = 0; y < other._height; y++) {
                        globalExplorationStatuses[x, y] |= other._tilesCoveredStatus[x, y];
                    }
                }
            }
            map._tilesCoveredStatus = globalExplorationStatuses.Clone() as bool[,];

            // Synchronize tile statuses
            var globalMap = new SlamMap.SlamTileStatus[others[0]._width, others[0]._height];
            for (int x = 0; x < others[0]._width; x++) {
                for (int y = 0; y < others[0]._height; y++) {
                    int slamX = x * 2;
                    int slamY = y * 2;
                    var status = newSlamStatuses[slamX, slamY];
                    status = AggregateStatusOptimistic(status, newSlamStatuses[slamX + 1, slamY]);
                    status = AggregateStatusOptimistic(status, newSlamStatuses[slamX + 1, slamY + 1]);
                    status = AggregateStatusOptimistic(status, newSlamStatuses[slamX, slamY + 1]);
                    globalMap[x, y] = status;
                }
            }

            map._optimisticTileStatuses = globalMap.Clone() as SlamMap.SlamTileStatus[,];
        }

        // Returns false only if the tile is known to be solid
        public bool IsPotentiallyExplorable(Vector2Int coordinate) {
            var withinBounds = coordinate.x >= 0 && coordinate.x < _width && coordinate.y >= 0 && coordinate.y < _height;
            // To avoid giving away information which the robot cannot know, tiles outside the map bounds are
            // considered explorable
            if (!withinBounds) return true;

            return (!_tilesCoveredStatus[coordinate.x, coordinate.y]) && GetSlamTileStatuses(coordinate).All(status => status != SlamMap.SlamTileStatus.Solid);
        }

        private List<SlamMap.SlamTileStatus> GetSlamTileStatuses(Vector2Int coordinate) {
            var slamCoord = coordinate * 2;
            return new List<SlamMap.SlamTileStatus>() {
                _slamMap.GetStatusOfTile(slamCoord),
                _slamMap.GetStatusOfTile(slamCoord + Vector2Int.right),
                _slamMap.GetStatusOfTile(slamCoord + Vector2Int.up),
                _slamMap.GetStatusOfTile(slamCoord + Vector2Int.up + Vector2Int.right),
            };
        }

        public void UpdateTile(Vector2Int courseCoord, SlamMap.SlamTileStatus observedStatus) {
            // If some sub-tile of the coarse tile is known to be solid, then new status does not matter
            // Other wise assign the new status (either open or solid)
            if (_optimisticTileStatuses[courseCoord.x, courseCoord.y] != SlamMap.SlamTileStatus.Solid)
                _optimisticTileStatuses[courseCoord.x, courseCoord.y] = observedStatus;
        }
    }

}