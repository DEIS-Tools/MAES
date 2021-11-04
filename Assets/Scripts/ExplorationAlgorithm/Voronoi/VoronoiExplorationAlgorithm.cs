using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Dora.Robot;
using Dora.Utilities;
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
        private int TILE_EXPLORATION_RANGE = 1;
        private int _currentTick = 0;

        private Vector2? _currentTarget = null;

        private VoronoiSearchPhase _currentSearchPhase;

        private enum VoronoiSearchPhase {
            SEARCH_MODE, // If not unexplored tiles within view 
            EXPLORE_MODE // Go to next unexplored tile
        }

        private struct VoronoiRegion {
            public readonly int RobotId;
            public readonly List<Vector2Int> Tiles;

            public VoronoiRegion(int robotId, List<Vector2Int> tiles) {
                RobotId = robotId;
                Tiles = tiles;
            }
        }

        public VoronoiExplorationAlgorithm(int randomSeed) {
            _random = new Random(randomSeed);
            _currentSearchPhase = VoronoiSearchPhase.EXPLORE_MODE;
        }

        public void UpdateLogic() {
            if (_currentTick % 20 == 0 && _robotController.GetStatus() == RobotStatus.Idle) {
                _robotController.Move(1);
            }
            
            UpdateAdjacentTilesToExplored();

            // Divide into voronoi regions with local robots
            if(ShouldRecalculate())
                RecalculateVoronoiRegions();
            
                // Find unexplored tiles
                // TODO: The voronoi region may be empty, if the robot is 1/4 the size of a slam tile and is located between tiles, but surrounded by other robots
                var myRegion = _localVoronoiRegions
                                    .DefaultIfEmpty(new VoronoiRegion())
                                    .First(k => k.RobotId == this._robotController.GetRobotID());
                var unexploredTiles = FindUnexploredTilesWithinView(myRegion);

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

        private void UpdateAdjacentTilesToExplored() {
            var currentPosition = this._robotController.GetSlamMap().GetCurrentPositionTile();
            var currentlyVisibleTiles = this._robotController.GetSlamMap().GetCurrentlyVisibleTiles();

            foreach (var visibleTile in currentlyVisibleTiles) {
                if (Geometry.DistanceBetween(currentPosition, visibleTile.Key) <= TILE_EXPLORATION_RANGE) {
                    if (!_exploredTiles.Contains(visibleTile.Key)) {
                        _exploredTiles.Add(visibleTile.Key);
                    }
                }
            }
        }

        private bool ShouldRecalculate() {
            return this._robotController.HasCollided() || this._robotController.GetStatus() == RobotStatus.Idle;
        }

        private void EnterSearchMode() {
            this._currentSearchPhase = VoronoiSearchPhase.SEARCH_MODE;
            
            var occlusionPoints = FindOcclusionPoints();

            if (occlusionPoints.Count != 0) {
                AttemptExpansionOfVoronoiRegion();
            }
            else {
                
            }
        }

        private void AttemptExpansionOfVoronoiRegion() {
            
        }

        private List<Vector2> FindOcclusionPoints() {
                


            return null;
        }

        private List<Vector2Int> FindUnexploredTilesWithinView(VoronoiRegion region) {
            var currentlyVisibleTiles = this._robotController.GetSlamMap().GetCurrentlyVisibleTiles();

            return currentlyVisibleTiles
                        .Where(e => !_exploredTiles.Contains(e.Key))
                        .Select(e => e.Key)
                        .ToList();
        }

        private void RecalculateVoronoiRegions() {
            // TODO: Voronoi regions should take line of sight into account?
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

            info.Append($"Search phase: {_currentSearchPhase}\n");
            info.Append($"Explored tiles: {String.Join(",", _exploredTiles)}\n");
            
            return info.ToString();
        }
    }
}