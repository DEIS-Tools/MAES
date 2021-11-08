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
using Random = System.Random;

namespace Dora.ExplorationAlgorithm.Voronoi {
    public class VoronoiExplorationAlgorithm : IExplorationAlgorithm {
        private IRobotController _robotController;
        private readonly Random _random;
        
        private List<VoronoiRegion> _localVoronoiRegions = new List<VoronoiRegion>();
        private bool _shouldRecalculateRegions = true;
        private int _voronoiMinimumRange = 10; // TODO assign dynamically according to constraints maybe?
        private List<Vector2Int> _exploredTiles = new List<Vector2Int>();
        private int _currentTick = 0;
        private Vector2Int? _closestOcclusionPoint = null;

        private Vector2? _currentTarget = null;

        private VoronoiSearchPhase _currentSearchPhase = VoronoiSearchPhase.EXPLORE_MODE;
        private readonly RobotConstraints _constraints;
        private readonly int _markExploredRangeInSlamTiles;
        private int _regionSize;
        private int _unexploredTilesInRegion;

        private enum VoronoiSearchPhase {
            SEARCH_MODE, // If not unexplored tiles within view 
            EXPLORE_MODE // Go to next unexplored tile
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
        }

        public void UpdateLogic() {
            if (_currentTick % 20 == 0 && _robotController.GetStatus() == RobotStatus.Idle) {
                _robotController.Move(1);
            }
            
            UpdateAdjacentTilesToExplored();
            
            // Divide into voronoi regions with local robots
            if (ShouldRecalculate()) {
                RecalculateVoronoiRegions();
            
                // Find unexplored tiles
                // TODO: The voronoi region may be empty, if the robot is 1/4 the size of a slam tile and is located between tiles, but surrounded by other robots
                var myRegion = _localVoronoiRegions
                    .DefaultIfEmpty(new VoronoiRegion())
                    .First(k => k.RobotId == this._robotController.GetRobotID());
                _regionSize = myRegion.Size();
                var unexploredTiles = FindUnexploredTilesWithinRegion(myRegion);
                _unexploredTilesInRegion = unexploredTiles.Count;

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
            return this._robotController.HasCollided()
                   || this._robotController.GetStatus() == RobotStatus.Idle
                   || _currentTick % 10 == 0;
        }

        private void EnterSearchMode() {
            this._currentSearchPhase = VoronoiSearchPhase.SEARCH_MODE;
            
            var occlusionPoint = FindClosestOcclusionPoint();

            if (!occlusionPoint.HasValue) {
                AttemptExpansionOfVoronoiRegion();
                _closestOcclusionPoint = null;
            }

            _closestOcclusionPoint = occlusionPoint.Value;
            _currentTarget = occlusionPoint.Value;
        }

        private void AttemptExpansionOfVoronoiRegion() {
            
        }

        private Vector2Int? FindClosestOcclusionPoint() {
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
                return null;
            
            possiblyOccludedEdges.Sort((c1, c2) => {
                var c1Distance = Geometry.DistanceBetween(robotPosition, c1);
                var c2Distance = Geometry.DistanceBetween(robotPosition, c2);
                return c1Distance.CompareTo(c2Distance);
            });

            return possiblyOccludedEdges[0];
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
                _localVoronoiRegions.Add(new VoronoiRegion(this._robotController.GetRobotID(), visibleTiles.Select(e => e.Key).ToList()));
                return;
            }
            
            // Find furthest away robot. Voronoi partition should include all robots within broadcast range
            nearbyRobots.Sort((r1, r2) => r1.Distance.CompareTo(r2.Distance));

            int voronoiMapRange = Math.Max(_voronoiMinimumRange, Convert.ToInt32(nearbyRobots[0].Distance));

            Dictionary<int, List<Vector2Int>> robotIdToClosestTilesMap = new Dictionary<int, List<Vector2Int>>();

            // Assign tiles to robots to create regions
            var currentlyVisibleTiles = this._robotController.GetSlamMap().GetCurrentlyVisibleTiles();
            for (int x = myPosition.x - voronoiMapRange; x < myPosition.x + voronoiMapRange; x++) {
                for (int y = myPosition.y - voronoiMapRange; y < myPosition.y + voronoiMapRange; y++) {
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