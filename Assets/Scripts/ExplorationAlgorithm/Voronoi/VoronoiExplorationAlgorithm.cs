using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.CompilerServices;
using System.Text;
using System.Text.RegularExpressions;
using Dora.Robot;
using Dora.Utilities;
using JetBrains.Annotations;
using UnityEditor.UI;
using UnityEngine;
using UnityEngine.Assertions;
using UnityEngine.WSA;
using Random = System.Random;

namespace Dora.ExplorationAlgorithm.Voronoi {
    public class VoronoiExplorationAlgorithm : IExplorationAlgorithm {
        private IRobotController _robotController;
        private readonly Random _random;
        
        private List<VoronoiRegion> _localVoronoiRegions = new List<VoronoiRegion>();
        private VoronoiRegion _currentRegion = new VoronoiRegion();
        private int _voronoiRegionMaxDistance; // TODO assign dynamically according to constraints maybe?
        private readonly List<Vector2Int> _exploredTiles = new List<Vector2Int>();
        private int _currentTick = 0;
        
        private VoronoiSearchPhase _currentSearchPhase = VoronoiSearchPhase.EXPLORE_MODE;
        private readonly RobotConstraints _constraints;
        private readonly int _markExploredRangeInSlamTiles;
        private readonly int EXPAND_VORONOI_RECALC_INTERVAL = 1;
        private readonly int SEARCH_MODE_RECALC_INTERVAL = 2;
        private readonly int EXPLORE_MODE_RECALC_INTERVAL = 10;

        // Debugging variables
        private Vector2Int? _closestOcclusionPoint = null;
        private Vector2? _currentTarget = null;
        private int _regionSize;
        private int _unexploredTilesInRegion;
        private int _recentlyVisitedOcclusionPoints;

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

        public VoronoiExplorationAlgorithm(int randomSeed, RobotConstraints constraints, int markExploredRangeInSlamTiles) {
            _random = new Random(randomSeed);
            _constraints = constraints;
            _markExploredRangeInSlamTiles = markExploredRangeInSlamTiles;
            _voronoiRegionMaxDistance = (int)constraints.SenseNearbyRobotRange;
        }

        public void UpdateLogic() {
            if (_currentTick % 5 == 0 && _robotController.GetStatus() == RobotStatus.Idle) {
                _robotController.Move(1);
            }
            
            UpdateAdjacentTilesToExplored();

            // Divide into voronoi regions with local robots
            if (ShouldRecalculate()) {
                Debug.Log("Recalculating voronoi");
                RecalculateVoronoiRegions();
                
                var unexploredTiles = FindUnexploredTilesWithinRegion(_currentRegion);
                _unexploredTilesInRegion = unexploredTiles.Count;
                _regionSize = _currentRegion.Size();

                if (unexploredTiles.Count != 0) {
                    this._currentSearchPhase = VoronoiSearchPhase.EXPLORE_MODE;
                    // Sorted by north, west, east, south
                    unexploredTiles.Sort((t1, t2) => {
                        if (t2.x.CompareTo(t1.x) == 0)
                            return t2.y.CompareTo(t1.y);
                        return t2.x.CompareTo(t1.x);
                    });

                    // Move to 
                    _currentTarget = new Vector2(unexploredTiles[0].x, unexploredTiles[0].y);
                }
                else {
                    EnterSearchMode();
                }
            }

            _currentTick++;
        }

        private void UpdateAdjacentTilesToExplored() {
            var currentPosition = this._robotController.GetSlamMap().GetCurrentPositionTile();
            var currentlyVisibleTiles = this._robotController.GetSlamMap().GetCurrentlyVisibleTiles();

            foreach (var visibleTile in currentlyVisibleTiles) {
                var distance = Geometry.DistanceBetween(currentPosition, visibleTile.Key);
                if (distance <= _markExploredRangeInSlamTiles) {
                    if (!_exploredTiles.Contains(visibleTile.Key)) {
                        _exploredTiles.Add(visibleTile.Key);
                    }
                }
            }
        }

        private bool ShouldRecalculate() {
            if (_robotController.HasCollided())
                return true;

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
            var occlusionPoints = FindClosestOcclusionPoints();

            if (occlusionPoints.Count == 0) {
                _currentSearchPhase = VoronoiSearchPhase.EXPAND_VORONOI;
                _currentTarget = FindClosestVoronoiBoundary();
                _closestOcclusionPoint = null;
            }
            else {
                this._currentSearchPhase = VoronoiSearchPhase.SEARCH_MODE;
                // Find occlusion point closest to robot
                var robotPosition = _robotController.GetSlamMap().GetCurrentPositionTile();
                occlusionPoints.Sort((c1, c2) => {
                    var c1Distance = Geometry.DistanceBetween(robotPosition, c1);
                    var c2Distance = Geometry.DistanceBetween(robotPosition, c2);
                    return c1Distance.CompareTo(c2Distance);
                });
                _closestOcclusionPoint = occlusionPoints[0];
                _currentTarget = occlusionPoints[0]; 
            }
        }

        private Vector2Int FindClosestVoronoiBoundary() {
            // Should move to closest voronoi boundary, that is not a wall
            var edgeTiles = FindEdgeTiles(_currentRegion.Tiles);
            
            var robotPosition = _robotController.GetSlamMap().GetCurrentPositionTile();
            edgeTiles.Sort((c1, c2) => {
                var c1Distance = Geometry.DistanceBetween(robotPosition, c1);
                var c2Distance = Geometry.DistanceBetween(robotPosition, c2);
                return c1Distance.CompareTo(c2Distance);
            });

            return edgeTiles[0];
        }

        private List<Vector2Int> FindClosestOcclusionPoints() {
            var visibleTilesMap = _robotController.GetSlamMap().GetCurrentlyVisibleTiles();
            var robotPosition = _robotController.GetSlamMap().GetCurrentPositionTile();
            
            // The surrounding edge of the visibility
            var visibilityEdge = FindEdgeTiles(visibleTilesMap.Select(e => e.Key).ToList());

            // Debug.Log("--------------------------");
            var furthestAwayTileDistance = 0f;
            foreach (var edge in visibilityEdge) {
                var range = Geometry.DistanceBetween(robotPosition, edge);
                if (range > furthestAwayTileDistance) {
                    furthestAwayTileDistance = range;
                }
                // Debug.Log($"Edge {edge.x},{edge.y}. Range: {Geometry.DistanceBetween(robotPosition, edge)}");
            }
            // Debug.Log("--------------------------");
            
            // Remove edges, that are as far away as our visibility range, since they cannot be occluded by anything.
            var possiblyOccludedEdges = visibilityEdge
                .Where(c => Geometry.DistanceBetween(robotPosition, c) < furthestAwayTileDistance - 4f)
                .Where(c => visibleTilesMap[c] != SlamMap.SlamTileStatus.Solid)
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
                            if (!countFlagList.Contains(neighTile)) {
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

        private List<Vector2Int> FindTilesInGroup(Vector2Int start, int maxDistFromRobot, Vector2Int robotPosition,
            DictionaryWithDefault<Vector2Int, bool> countedTiles) {
            var group = new List<Vector2Int>();
            Queue<Vector2Int> queue = new Queue<Vector2Int>();
            queue.Enqueue(start);
            countedTiles[start] = true;

            while (queue.Count > 0) {
                Vector2Int tile = queue.Dequeue();
                group.Add(tile);

                for (int x = tile.x - 1; x <= tile.x + 1; x++) {
                    for (int y = tile.y - 1; y <= tile.y + 1; y++) {
                        if (y == tile.y || x == tile.x) {
                            var adjTile = new Vector2Int(x, y);
                            
                            if (!countedTiles[adjTile] && Math.Ceiling(Geometry.DistanceBetween(robotPosition, adjTile)) < maxDistFromRobot) {
                                queue.Enqueue(new Vector2Int(x, y));
                            }
                            countedTiles[adjTile] = true;
                        }
                    }
                }
            }

            return group;
        }

        private List<Vector2Int> FindUnexploredTilesWithinRegion(VoronoiRegion region) {
            // TODO: The voronoi region may be empty, if the robot is 1/4 the size of a slam tile and is located between tiles, but surrounded by other robots
            if (region.IsEmpty()) return new List<Vector2Int>();

            return region.Tiles.Where(e => !_exploredTiles.Contains(e)).ToList();
        }

        private void RecalculateVoronoiRegions() {
            _localVoronoiRegions = new List<VoronoiRegion>();
            
            var nearbyRobots = _robotController.SenseNearbyRobots();
            var myPosition = _robotController.GetSlamMap().GetCurrentPositionTile();

            // If no near robots, all visible tiles are assigned to my own region
            if (nearbyRobots.Count == 0) {
                var visibleTiles = _robotController.GetSlamMap().GetCurrentlyVisibleTiles();
                var region = new VoronoiRegion(this._robotController.GetRobotID(),
                    visibleTiles.Select(e => e.Key)
                        .ToList());
                _localVoronoiRegions.Add(region);
                _currentRegion = region;
                return;
            }
            
            // Find furthest away robot. Voronoi partition should include all robots within broadcast range
            nearbyRobots.Sort((r1, r2) => r1.Distance.CompareTo(r2.Distance));

            Dictionary<int, List<Vector2Int>> robotIdToClosestTilesMap = new Dictionary<int, List<Vector2Int>>();
            
            // Assign tiles to robots to create regions
            var currentlyVisibleTiles = this._robotController.GetSlamMap().GetCurrentlyVisibleTiles();
            for (int x = myPosition.x - _voronoiRegionMaxDistance; x < myPosition.x + _voronoiRegionMaxDistance; x++) {
                for (int y = myPosition.y - _voronoiRegionMaxDistance; y < myPosition.y + _voronoiRegionMaxDistance; y++) {
                    // Only go through tiles within line of sight of the robot
                    if (currentlyVisibleTiles.ContainsKey(new Vector2Int(x, y))) {
                        var tilePosition = new Vector2Int(x, y);
                        float bestDistance = Utilities.Geometry.DistanceBetween(myPosition, tilePosition);
                        int bestRobotId = this._robotController.GetRobotID();
                        foreach (var robot in nearbyRobots) {
                            var otherPosition = robot.GetRelativePosition(myPosition, this._robotController.GetSlamMap().getRobotAngleDeg());
                            float distance = Utilities.Geometry.DistanceBetween(otherPosition, tilePosition);
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
        
        public void SetController(Robot2DController controller) {
            this._robotController = controller;
        }

        public string GetDebugInfo() {
            var info = new StringBuilder();
            
            if (_currentTarget.HasValue)
                info.Append($"Voronoi current target: x:{_currentTarget.Value.x}, y:{_currentTarget.Value.y}\n");
            else info.Append("Voronoi has no target\n");
            
            if (_closestOcclusionPoint.HasValue)
                info.Append($"Closest occlusion point: x:{_closestOcclusionPoint.Value.x}, y:{_closestOcclusionPoint.Value.y}\n");
            else info.Append("No closest occlusion point\n");

            info.Append($"Search phase: {_currentSearchPhase}\n");
            info.Append($"Current Region size: {_regionSize}\n");
            info.Append($"Unexplored tiles in region: {_unexploredTilesInRegion}\n");
            info.Append($"Explored tiles: {_exploredTiles.Count}\n");
            // info.Append($"Explored tiles: {String.Join(",", _exploredTiles)}\n");
            
            
            return info.ToString();
        }
    }
}