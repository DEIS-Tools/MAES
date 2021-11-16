using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Linq;
using System.Runtime.CompilerServices;
using System.Text;
using System.Text.RegularExpressions;
using Dora.Robot;
using Dora.Utilities;
using JetBrains.Annotations;
using UnityEditor;
using UnityEditor.UI;
using UnityEngine;
using UnityEngine.Assertions;
using UnityEngine.WSA;
using Debug = UnityEngine.Debug;
using Random = System.Random;

namespace Dora.ExplorationAlgorithm.Voronoi {
    public class VoronoiExplorationAlgorithm : IExplorationAlgorithm {
        private IRobotController _robotController;
        private readonly Random _random;
        
        private List<VoronoiRegion> _localVoronoiRegions = new List<VoronoiRegion>();
        private VoronoiRegion _currentRegion = new VoronoiRegion();
        private int _voronoiRegionMaxDistance; // Measured in coarse tiles
        private readonly Dictionary<Vector2Int, bool> _isExploredMap = new Dictionary<Vector2Int, bool>();
        private int _currentTick = 0;
        
        private VoronoiSearchPhase _currentSearchPhase = VoronoiSearchPhase.EXPLORE_MODE;
        private readonly RobotConstraints _constraints;
        private readonly float _markExploredRangeInSlamTiles;
        private readonly int EXPAND_VORONOI_RECALC_INTERVAL = 40;
        private readonly int SEARCH_MODE_RECALC_INTERVAL = 40;
        private readonly int EXPLORE_MODE_RECALC_INTERVAL = 50;
        private readonly float SELECT_FURTHEST_VORONOI_BOUNDARY_CHANCE = 0.3f;
        private readonly List<(Vector2Int, int)> _coarseOcclusionPointsVisitedThisSearchMode = new List<(Vector2Int, int)>();
        private List<(Vector2Int, bool)> _currentTargetPath = null; // bool represents, if it has been visited or not.
        private Vector2Int? _currentPartialMovementTarget = null;
        private readonly float DISTANCE_BETWEEN_SAME_OCC_POINT = 2f; // If two occlusion points are closer than this, they are the same
        private readonly float VORONOI_BOUNDARY_EQUAL_DISTANCE_DELTA = 2f;
        private VoronoiHeuristic _heuristic;
        private UnexploredTilesComparer _unexploredTilesComparer;

        private delegate int UnexploredTilesComparer(Vector2Int c1, Vector2Int c2);

        // Debugging variables
        private Vector2Int? _closestOcclusionPoint = null;
        private int _regionSizeCoarseTiles;
        private int _unexploredTilesInRegion;


        private enum VoronoiHeuristic {
            NORTH_EAST,
            EAST_SOUTH,
            SOUTH_WEST,
            WEST_NORTH
        }

        private enum VoronoiSearchPhase {
            SEARCH_MODE, // If not unexplored tiles within view 
            EXPLORE_MODE, // Go to next unexplored tile
            EXPAND_VORONOI
        }

        private struct VoronoiRegion {
            public readonly int RobotId;
            public readonly List<Vector2Int> Tiles;

            public bool IsEmpty() {
                return Tiles == null || Tiles.Count == 0;
            }

            public int Size() {
                if (IsEmpty()) return 0;
                return Tiles.Count;
            }

            public VoronoiRegion(int robotId, List<Vector2Int> tiles) {
                RobotId = robotId;
                Tiles = tiles;
            }
        }

        public VoronoiExplorationAlgorithm(int randomSeed, RobotConstraints constraints, float markExploredRangeInSlamTiles) {
            _random = new Random(randomSeed);
            _constraints = constraints;
            _markExploredRangeInSlamTiles = markExploredRangeInSlamTiles;
            _voronoiRegionMaxDistance = (int)constraints.SenseNearbyRobotRange;
        }
        
        public void SetController(Robot2DController controller) {
            this._robotController = controller;
            _heuristic = (VoronoiHeuristic)(_robotController.GetRobotID() % 4);
            _unexploredTilesComparer = GetSortingFunction(_heuristic);
        }

        public void UpdateLogic() {
            UpdateExploredStatusOfTiles();

            // Divide into voronoi regions with local robots
            if (ShouldRecalculate()) {
                // Debug.Log("Recalculating voronoi");
                RecalculateVoronoiRegions();
                
                var unexploredTiles = FindUnexploredTilesWithinRegion(_currentRegion);
                _unexploredTilesInRegion = unexploredTiles.Count;
                _regionSizeCoarseTiles = _currentRegion.Size();

                if (unexploredTiles.Count != 0) {
                    this._currentSearchPhase = VoronoiSearchPhase.EXPLORE_MODE;
                    _coarseOcclusionPointsVisitedThisSearchMode.Clear();
                    // _currentTargetPath = null;
                    // _currentPartialMovementTarget = null;
                    // Sorted by north, west, east, south 
                    unexploredTiles.Sort((c1, c2) => _unexploredTilesComparer(c1, c2));
                    
                    // Movement target is the best tile, where a path can be found
                    var target = unexploredTiles[0];
                    SetCurrentMovementTarget(target);
                }
                else {
                    EnterSearchMode();
                }
            }

            // Make the robot follow the current path.
            if ((!IsDoneWithCurrentPath() && _robotController.GetStatus() == RobotStatus.Idle)) {
                var nextStep = GetNextStep();
                if (nextStep != null) {
                    _currentPartialMovementTarget = nextStep.Value;
                    var relativePosition = _robotController.GetSlamMap().GetCoarseMap()
                        .GetTileCenterRelativePosition(_currentPartialMovementTarget.Value);
                    
                    // Find delta
                    float delta = 1.0f;
                    if (Math.Abs(relativePosition.RelativeAngle) <= delta) {
                        _robotController.Move((relativePosition.Distance));
                    }
                    else {
                        _robotController.Rotate(relativePosition.RelativeAngle);
                    }
                }
                else {
                    Debug.Log("Next step == null");
                }
                    
            }

            _currentTick++;
        }

        private UnexploredTilesComparer GetSortingFunction(VoronoiHeuristic heuristic) {
            switch (heuristic) {
                case VoronoiHeuristic.NORTH_EAST:
                    return (c1, c2) => {
                        if (c2.y.CompareTo(c1.y) == 0)
                            return -c2.x.CompareTo(c1.x); // Higher x = east
                        return -c2.y.CompareTo(c1.y); // Higher y = north
                    };
                case VoronoiHeuristic.EAST_SOUTH:
                    return (c1, c2) => {
                        if (c2.x.CompareTo(c1.x) == 0)
                            return c2.y.CompareTo(c1.y); // lower y = south
                        return -c2.x.CompareTo(c1.x); // higher x = east
                    };
                case VoronoiHeuristic.SOUTH_WEST:
                    return (c1, c2) => {
                        if (c2.y.CompareTo(c1.y) == 0)
                            return c2.x.CompareTo(c1.x); // lower x = west
                        return c2.y.CompareTo(c1.y); // lower y = south
                    };
                case VoronoiHeuristic.WEST_NORTH:
                    return (c1, c2) => {
                        if (c2.x.CompareTo(c1.x) == 0)
                            return -c2.y.CompareTo(c1.y); // higher y = north
                        return c2.x.CompareTo(c1.x); // higher x = east
                    };
                default:
                    throw new Exception("Could not find sorting function for voronoi heuristic.");
            }
        }

        private Vector2Int? GetNextStep() {
            if (IsDoneWithCurrentPath()) return null;
            else {
                return _currentTargetPath.First(e => e.Item2 == false).Item1;
            }
        }

        private void UpdateExploredStatusOfTiles() {
            var currentPosition = this._robotController.GetSlamMap().GetCoarseMap().GetApproximatePosition();
            var currentlyVisibleTiles = this._robotController.GetSlamMap().GetCurrentlyVisibleTiles();
            var currentlyVisibleCoarseTiles = this._robotController.GetSlamMap().GetCoarseMap()
                .FromSlamMapCoordinates(currentlyVisibleTiles.Keys.ToList());

            foreach (var visibleTile in currentlyVisibleCoarseTiles) {
                var distance = Geometry.DistanceBetween(currentPosition, visibleTile);
                if (distance <= _markExploredRangeInSlamTiles) {
                    if (!_isExploredMap.ContainsKey(visibleTile)) {
                        _isExploredMap[visibleTile] = true;
                    }
                }
            }

            // Check if movement target reached
            if (!IsDoneWithCurrentPath()) {
                if (_currentTargetPath != null && _currentPartialMovementTarget != null) {
                    // Have we reached the next partial goal?
                    var robotCoarseTile = _robotController.GetSlamMap().GetCoarseMap().GetCurrentTile();
                    if (robotCoarseTile.Equals(_currentPartialMovementTarget.Value)) {
                        // Mark the current partial movement target as visited
                        _currentTargetPath = _currentTargetPath.Select(e => {
                            if (e.Item1.Equals(robotCoarseTile))
                                return (e.Item1, true);
                            return e;
                        }).ToList();
                        if (_currentSearchPhase == VoronoiSearchPhase.SEARCH_MODE) {
                            _coarseOcclusionPointsVisitedThisSearchMode.Add((_currentPartialMovementTarget.Value, _currentTick));
                        }
                    }
                }
            }
        }

        private bool ShouldRecalculate() {
            if (_robotController.IsCurrentlyColliding()) {
                _robotController.StopCurrentTask();
                return true;
            }
                

            if (IsDoneWithCurrentPath()) {
                _currentTargetPath = null;
                return true;
            }

            switch (_currentSearchPhase) {
                case VoronoiSearchPhase.SEARCH_MODE:
                    return _currentTick % SEARCH_MODE_RECALC_INTERVAL == 0;
                case VoronoiSearchPhase.EXPLORE_MODE:
                    return _currentTick % EXPLORE_MODE_RECALC_INTERVAL == 0;
                case VoronoiSearchPhase.EXPAND_VORONOI:
                    return _currentTick % EXPAND_VORONOI_RECALC_INTERVAL == 0;
            }

            return false;
        }

        private void EnterSearchMode() {
            var coarseMap = _robotController.GetSlamMap().GetCoarseMap();
            var coarseOcclusionPoints = FindClosestOcclusionPoints();
            // Occlusion points close to something visited recently will not be visited
            var coarseOcclusionPointsNotVisitedThisSearchMode = new List<Vector2Int>();
            foreach (var op in coarseOcclusionPoints) {
                foreach (var visitedOccPo in _coarseOcclusionPointsVisitedThisSearchMode) {
                    if (Geometry.DistanceBetween(op, visitedOccPo.Item1) > DISTANCE_BETWEEN_SAME_OCC_POINT) {
                        coarseOcclusionPointsNotVisitedThisSearchMode.Add(op);
                    }
                }
            }
            
            // If we have some unvisited occlusion points, visit them. 
            if (coarseOcclusionPointsNotVisitedThisSearchMode.Count > 0) {
                this._currentSearchPhase = VoronoiSearchPhase.SEARCH_MODE;
                // Find occlusion point closest to robot
                var robotPosition = coarseMap.GetApproximatePosition();
                coarseOcclusionPointsNotVisitedThisSearchMode.Sort((c1, c2) => {
                    var c1Distance = Geometry.DistanceBetween(robotPosition, c1);
                    var c2Distance = Geometry.DistanceBetween(robotPosition, c2);
                    return c1Distance.CompareTo(c2Distance);
                });
                _closestOcclusionPoint = coarseOcclusionPointsNotVisitedThisSearchMode[0];
                SetCurrentMovementTarget(coarseOcclusionPointsNotVisitedThisSearchMode[0]);
            }
            else if (coarseOcclusionPointsNotVisitedThisSearchMode.Count == 0) {
                // If we have visited all occlusion points, but we still see several
                // Visit least recently visited occlusion point
                if (_coarseOcclusionPointsVisitedThisSearchMode.Count > 1) {
                    this._currentSearchPhase = VoronoiSearchPhase.SEARCH_MODE;
                    _coarseOcclusionPointsVisitedThisSearchMode.Sort((e1, e2) => e1.Item2.CompareTo(e2.Item2));
                    var leastRecentlyVisitedOcclusionPoint = _coarseOcclusionPointsVisitedThisSearchMode[0];
                    SetCurrentMovementTarget(leastRecentlyVisitedOcclusionPoint.Item1);
                    _closestOcclusionPoint = leastRecentlyVisitedOcclusionPoint.Item1;
                }
                // If we see no occlusion points or the only occlusion point is already visited, expand region
                else if (_coarseOcclusionPointsVisitedThisSearchMode.Count == 0 || (_coarseOcclusionPointsVisitedThisSearchMode.Count == 1 && coarseOcclusionPointsNotVisitedThisSearchMode.Count == 0)) {
                    _currentSearchPhase = VoronoiSearchPhase.EXPAND_VORONOI;
                    var closestVoronoiBoundary = FindClosestVoronoiBoundary();
                    SetCurrentMovementTarget(closestVoronoiBoundary);
                    _closestOcclusionPoint = null;
                }
            }
        }

        private bool IsDoneWithCurrentPath() {
            if (_currentTargetPath == null) return true;
            return _currentTargetPath.TrueForAll(e => e.Item2 == true);
        }

        private void SetCurrentMovementTarget(Vector2Int sortedTargetList) {
            // Find a the course coordinate and generate path
            var robotCoarseTile = _robotController.GetSlamMap().GetCoarseMap().GetCurrentTile();
            var path = _robotController.GetSlamMap().GetCoarseMap().GetPath(sortedTargetList, true);

            if (path == null) {
                Debug.Log($"Could not find path between {robotCoarseTile} and {sortedTargetList}");
            }
            else {
                _currentTargetPath = path.Select(e => (e, false)).ToList(); // All points on path are not visited, i.e. false
                if (_currentTargetPath[0].Item1.Equals(robotCoarseTile) &&
                    !_robotController.HasCollidedSinceLastLogicTick()) {
                    _currentTargetPath[0] = (_currentTargetPath[0].Item1, true);
                }
            }
        }

        private Vector2Int FindClosestVoronoiBoundary() {
            var coarseMap = _robotController.GetSlamMap().GetCoarseMap();
            // Should move to closest voronoi boundary, that is not a wall
            var coarseEdgeTiles = FindEdgeTiles(_currentRegion.Tiles);
            coarseEdgeTiles = coarseEdgeTiles.Distinct().ToList(); // Remove possible duplicate

            // Filter away edge tiles near obstacles
            var openCoarseEdgeTiles = coarseEdgeTiles.Where(e => !coarseMap.IsOptimisticSolid(e)).ToList();
            // If no voronoi boundary is available without an obstacle, just take one with an obstacle
            if (openCoarseEdgeTiles.Count > 0)
                coarseEdgeTiles = openCoarseEdgeTiles;
            
            var robotPosition = coarseMap.GetApproximatePosition();
            

            var chance = _random.NextDouble();
            if (chance < SELECT_FURTHEST_VORONOI_BOUNDARY_CHANCE) {
                // Sort by furthest from robot
                coarseEdgeTiles.Sort((c1, c2) => {
                    var c1Distance = Geometry.DistanceBetween(robotPosition, c1);
                    var c2Distance = Geometry.DistanceBetween(robotPosition, c2);
                    return c2Distance.CompareTo(c1Distance);
                });
            }
            else {
                // Sort by closest to robot
                coarseEdgeTiles.Sort((c1, c2) => {
                    var c1Distance = Geometry.DistanceBetween(robotPosition, c1);
                    var c2Distance = Geometry.DistanceBetween(robotPosition, c2);
                    return c1Distance.CompareTo(c2Distance);
                });
            }

            // Find all tiles with a distance within delta and assume they are equally far away
            var closestOrFurthestDistance = Geometry.DistanceBetween(robotPosition, coarseEdgeTiles[0]);
            var candidates = coarseEdgeTiles.Where(e =>
                Mathf.Abs(Geometry.DistanceBetween(e, robotPosition) - closestOrFurthestDistance) < VORONOI_BOUNDARY_EQUAL_DISTANCE_DELTA).ToList();
            
            // Filter away anything close to the occlusion points visited this search mode
            var filteredCandidates = candidates.Where(e => {
                foreach (var occPo in _coarseOcclusionPointsVisitedThisSearchMode) {
                    if (Geometry.DistanceBetween(e, occPo.Item1) > DISTANCE_BETWEEN_SAME_OCC_POINT)
                        return false;
                }

                return true;
            }).ToList();

            if (filteredCandidates.Count > 0)
                candidates = filteredCandidates;
                
            // Take random candidate
            var candidateIndex = _random.Next(candidates.Count);

            return candidates[candidateIndex];
        }

        private List<Vector2Int> FindClosestOcclusionPoints() {
            var coarseMap = _robotController.GetSlamMap().GetCoarseMap();
            var visibleTilesMaps = _robotController.GetSlamMap().GetCurrentlyVisibleTiles();
            var visibleCoarseTiles = coarseMap.FromSlamMapCoordinates(visibleTilesMaps.Keys.ToList());

            var robotPosition = coarseMap.GetApproximatePosition();
            
            // The surrounding edge of the visibility
            var edgeTiles = FindEdgeTiles(visibleCoarseTiles);
            
            // Filter out edge tiles, that are walls. They can't show anything new
            var nonSolidEdgeTiles = edgeTiles.Where(e => !coarseMap.IsOptimisticSolid(e)).ToList();

            // Debug.Log("--------------------------");
            var furthestAwayTileDistance = 0f;
            foreach (var edge in nonSolidEdgeTiles) {
                var range = Geometry.DistanceBetween(robotPosition, edge);
                if (range > furthestAwayTileDistance) {
                    furthestAwayTileDistance = range;
                }
                // Debug.Log($"Edge {edge.x},{edge.y}. Range: {Geometry.DistanceBetween(robotPosition, edge)}");
            }
            // Debug.Log("--------------------------");
            
            // Remove edges, that are as far away as our visibility range, since they cannot be occluded by anything.
            var possiblyOccludedEdges = nonSolidEdgeTiles
                .Where(c => Geometry.DistanceBetween(robotPosition, c) < furthestAwayTileDistance - 2f)
                .ToList();

            if (possiblyOccludedEdges.Count == 0)
                return new List<Vector2Int>();

            // Find group of open edges (possibly occluded by walls)
            var edgeTileGroups = FindGroupedTiles(possiblyOccludedEdges);

            // Take closest point from all groups
            var occlusionPoints = new List<Vector2Int>();
            foreach (var edgeGroup in edgeTileGroups) {
                edgeGroup.Sort((c1, c2) => {
                    var c1Distance = Geometry.DistanceBetween(robotPosition, c1);
                    var c2Distance = Geometry.DistanceBetween(robotPosition, c2);
                    return c1Distance.CompareTo(c2Distance);
                });
                occlusionPoints.Add(edgeGroup[0]);
            }
            
            return occlusionPoints;
        }

        private List<List<Vector2Int>> FindGroupedTiles(List<Vector2Int> allTiles) {
            var res = new List<List<Vector2Int>>();

            // No tiles are accounted for in the beginning
            var counter = new Dictionary<Vector2Int, bool>();
            foreach(var tile in allTiles) counter[tile] = false;

            // Continue while some tiles have not yet been counted
            while (counter.ContainsValue(false)) {
                var next = counter.First(e => e.Value == false);
                var connectedTiles = GetConnectedTiles(next.Key, allTiles);
                foreach (var tile in connectedTiles) counter[tile] = true;
                res.Add(connectedTiles);
            }

            var totalTiles = allTiles.Count;
            var totalAfter = res.Aggregate(0, (e1, e2) => e1 + e2.Count);
            Assert.IsTrue(totalTiles == totalAfter);

            return res;
        }

        private List<Vector2Int> GetConnectedTiles(Vector2Int startTile, List<Vector2Int> allTiles) {
            var res = new List<Vector2Int>();
            var countFlagList = new List<Vector2Int>();
            Queue<Vector2Int> queue = new Queue<Vector2Int>();
            queue.Enqueue(startTile);
            countFlagList.Add(startTile);

            while (queue.Count > 0) {
                var tile = queue.Dequeue();
                res.Add(tile);

                for (int x = tile.x - 1; x <= tile.x + 1; x++) {
                    for (int y = tile.y - 1; y <= tile.y + 1; y++) {
                        if (x == tile.x || y == tile.y) { // Diagonal does not count.
                            var neighTile = new Vector2Int(x, y);
                            if (!countFlagList.Contains(neighTile) && allTiles.Contains(neighTile)) {
                                queue.Enqueue(neighTile);
                                countFlagList.Add(neighTile);
                            }
                        }
                    }
                }
            }

            return res;
        }

        private List<Vector2Int> FindEdgeTiles(List<Vector2Int> tiles) {
            // An edge is any tile, where a neighbor is missing in the set of tiles.
            var edgeTiles = new List<Vector2Int>();

            foreach (var tile in tiles) {
                var isEdge = false;
                for (int x = tile.x - 1; x <= tile.x + 1; x++) {
                    for (int y = tile.y - 1; y <= tile.y + 1; y++) {
                        if (x == tile.x || y == tile.y) {
                            var neighbour = new Vector2Int(x, y);
                            if (!tiles.Contains(neighbour)) isEdge = true;
                        }
                    }
                }
                
                if(isEdge) edgeTiles.Add(tile);
            }

            return edgeTiles;
        }

        private List<Vector2Int> FindUnexploredTilesWithinRegion(VoronoiRegion region) {
            // TODO: The voronoi region may be empty, if the robot is 1/4 the size of a slam tile and is located between tiles, but surrounded by other robots
            if (region.IsEmpty()) return new List<Vector2Int>();

            var coarseMap = _robotController.GetSlamMap().GetCoarseMap();
            
            // since the path finder uses bigger tiles than the slam map, we can risk looking for a tile without a known path
            var unExploredInRegion = region.Tiles.Where(e => !_isExploredMap.ContainsKey(e) || _isExploredMap[e] == false).ToList();

            unExploredInRegion =
                unExploredInRegion
                    .Where(e => coarseMap.GetTileStatus(e) != SlamMap.SlamTileStatus.Solid)
                    .ToList();
            
            // Filter out any slam tiles right next to a wall.
            return unExploredInRegion;
        }
        
        private void RecalculateVoronoiRegions() {
            var coarseMap = _robotController.GetSlamMap().GetCoarseMap();
            _localVoronoiRegions = new List<VoronoiRegion>();
            
            var nearbyRobots = _robotController.SenseNearbyRobots();
            var myPosition = _robotController.GetSlamMap().GetCoarseMap().GetCurrentTile();

            // If no near robots, all visible tiles are assigned to my own region
            if (nearbyRobots.Count == 0) {
                var visibleSlamTiles = _robotController.GetSlamMap().GetCurrentlyVisibleTiles();
                var visibleCoarseTiles = coarseMap.FromSlamMapCoordinates(visibleSlamTiles.Keys.ToList());
                var region = new VoronoiRegion(this._robotController.GetRobotID(), visibleCoarseTiles);
                _localVoronoiRegions.Add(region);
                _currentRegion = region;
                return;
            }
            
            // Find furthest away robot. Voronoi partition should include all robots within broadcast range
            nearbyRobots.Sort((r1, r2) => r1.Distance.CompareTo(r2.Distance));

            Dictionary<int, List<Vector2Int>> robotIdToClosestTilesMap = new Dictionary<int, List<Vector2Int>>();
            
            // Assign tiles to robots to create regions
            var currentlyVisibleTiles = this._robotController.GetSlamMap().GetCurrentlyVisibleTiles();
            var currentlyVisibleCoarseTiles = coarseMap.FromSlamMapCoordinates(currentlyVisibleTiles.Keys.ToList());
            for (int x = myPosition.x - _voronoiRegionMaxDistance; x < myPosition.x + _voronoiRegionMaxDistance; x++) {
                for (int y = myPosition.y - _voronoiRegionMaxDistance; y < myPosition.y + _voronoiRegionMaxDistance; y++) {
                    // Only go through tiles within line of sight of the robot
                    if (currentlyVisibleCoarseTiles.Contains(new Vector2Int(x, y))) {
                        var tilePosition = new Vector2Int(x, y);
                        float bestDistance = Geometry.DistanceBetween(myPosition, tilePosition);
                        int bestRobotId = this._robotController.GetRobotID();
                        foreach (var robot in nearbyRobots) {
                            var otherPosition = robot.GetRelativePosition(myPosition, this._robotController.GetSlamMap().GetRobotAngleDeg());
                            float distance = Geometry.DistanceBetween(otherPosition, tilePosition);
                            if (distance < bestDistance) {
                                bestDistance = distance;
                                bestRobotId = robot.item;
                            }
                        }

                        if (!robotIdToClosestTilesMap.Keys.Contains(bestRobotId))
                            robotIdToClosestTilesMap[bestRobotId] = new List<Vector2Int>();
                        robotIdToClosestTilesMap[bestRobotId].Add(tilePosition);
                    }
                }
            }

            foreach (var kv in robotIdToClosestTilesMap) {
                this._localVoronoiRegions.Add(new VoronoiRegion(kv.Key, kv.Value));
            }
            
            _currentRegion = _localVoronoiRegions
                .DefaultIfEmpty(new VoronoiRegion())
                .First(k => k.RobotId == this._robotController.GetRobotID());
        }
        
        
        public object SaveState() {
            throw new System.NotImplementedException();
        }

        public void RestoreState(object stateInfo) {
            throw new System.NotImplementedException();
        }
        
        

        public string GetDebugInfo() {
            var info = new StringBuilder();
            
            if (_currentTargetPath != null && _currentTargetPath.Count > 0) {
                var finalTarget = _currentTargetPath[_currentTargetPath.Count - 1];
                info.Append($"Current final target: x:{finalTarget.Item1.x}, y:{finalTarget.Item1.y}\n");
            }
            else info.Append("Voronoi has no final target\n");
            
            if (_currentPartialMovementTarget.HasValue)
                info.Append($"Current partial target: x:{_currentPartialMovementTarget.Value.x}, y:{_currentPartialMovementTarget.Value.y}\n");
            else info.Append("Voronoi has no partial target\n");
            
            if (_closestOcclusionPoint.HasValue)
                info.Append($"Closest occlusion point slamtile: x:{_closestOcclusionPoint.Value.x}, y:{_closestOcclusionPoint.Value.y}\n");
            else info.Append("No closest occlusion point\n");

            info.Append($"Search phase: {_currentSearchPhase}\n");
            info.Append($"Current Region size: {_regionSizeCoarseTiles}\n");
            info.Append($"Unexplored tiles in region: {_unexploredTilesInRegion}\n");
            info.Append($"Explored tiles: {_isExploredMap.Count}\n");

            if (_currentTargetPath == null) 
                info.Append($"No path to target found\n");
            else
                info.Append($"Path tiles: {String.Join(",", _currentTargetPath)}\n");
            // info.Append($"Explored tiles: {String.Join(",", _exploredTiles)}\n");
            
            
            return info.ToString();
        }
    }
}