using System;
using System.Collections.Generic;
using System.Linq;
using Dora.Robot;
using UnityEngine;
using Random = System.Random;

namespace Dora.ExplorationAlgorithm.Voronoi {
    public class VoronoiExplorationAlgorithm : IExplorationAlgorithm {
        private IRobotController _robotController;
        private readonly Random _random;
        
        private List<VoronoiRegion> _localVoronoiRegions;
        private bool _shouldRecalculateRegions = true;
        private int _voronoiMinimumRange = 10; // TODO assign dynamically according to constraints maybe?

        private Vector2? _currentTarget = null;

        private struct VoronoiRegion {
            public readonly int RobotId;
            public readonly List<Vector2> Tiles;

            public VoronoiRegion(int robotId, List<Vector2> tiles) {
                RobotId = robotId;
                Tiles = tiles;
            }
        }

        public VoronoiExplorationAlgorithm(int randomSeed) {
            _random = new Random(randomSeed);
        }

        public void UpdateLogic() {
            // Divide into voronoi regions with local robots
            if(ShouldRecalculate())
                RecalculateVoronoiRegions();
            
                // Find unexplored tiles
                // TODO: The voronoi region may be empty, if the robot is 1/4 the size of a slam tile and is located between tiles, but surrounded by other robots
                var myRegion = _localVoronoiRegions
                                    .DefaultIfEmpty(new VoronoiRegion())
                                    .First(k => k.RobotId == this._robotController.GetRobotID());
                var unexploredTiles = FindUnexploredTiles(myRegion);
                
                if (unexploredTiles.Count == 0)
                    EnterSearchMode();
                
                // Sorted by north, west, east, south
                unexploredTiles.Sort((t1, t2) => {
                    if (t2.x.CompareTo(t1.x) == 0)
                        return t2.y.CompareTo(t1.y);
                    return t2.x.CompareTo(t1.x);
                });
                
                // Move to 
                _currentTarget = unexploredTiles[0];
                
        }

        private bool ShouldRecalculate() {
            return this._robotController.HasCollided() || this._robotController.GetStatus() == RobotStatus.Idle;
        }

        private void EnterSearchMode() {
            _shouldRecalculateRegions = true;
        }

        private List<Vector2> FindUnexploredTiles(VoronoiRegion region) {
            var exploredTilesCoords = this._robotController.GetSlamMap().GetExploredTiles()
                                                    .Select(e => e.Item1)
                                                    .ToList();

            // Remove all already explored tiles from region.
            return region.Tiles.Where(e => !exploredTilesCoords.Contains(e)).ToList();
        }

        private void RecalculateVoronoiRegions() {
            // TODO: Voronoi regions should take line of sight into account?
            _localVoronoiRegions = new List<VoronoiRegion>();
            
            var nearbyRobots = _robotController.SenseNearbyRobots();

            var myPosition = _robotController.GetSlamMap().GetApproxPosition();

            if (nearbyRobots.Count == 0) {
                var tiles = new List<Vector2>();
                for (int x = (int)(myPosition.x - _voronoiMinimumRange); x < (int)(myPosition.x + _voronoiMinimumRange); x++) {
                    for (int y = (int)(myPosition.y - _voronoiMinimumRange); y < (int)(myPosition.y + _voronoiMinimumRange); y++) {
                        tiles.Add(new Vector2(x, y));
                    }
                }

                _localVoronoiRegions.Add(new VoronoiRegion(this._robotController.GetRobotID(), tiles));
                return;
            }
            
            // Find furthest away robot. Voronoi partition should include all robots within broadcast range
            nearbyRobots.Sort((r1, r2) => r1.Distance.CompareTo(r2.Distance));

            float voronoiMapRange = Mathf.Max(_voronoiMinimumRange,nearbyRobots[0].Distance);

            Dictionary<int, List<Vector2>> robotIdToClosestTilesMap = new Dictionary<int, List<Vector2>>();

            // Assign tiles to robots to create regions
            for (int x = (int)(myPosition.x - voronoiMapRange); x < (int)(myPosition.x + voronoiMapRange); x++) {
                for (int y = (int)(myPosition.y - voronoiMapRange); y < (int)(myPosition.y + voronoiMapRange); y++) {
                    var tilePosition = new Vector2(x, y);
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
                        robotIdToClosestTilesMap[bestRobotId] = new List<Vector2>();
                    robotIdToClosestTilesMap[bestRobotId].Add(tilePosition);
                }
            }

            foreach (var kv in robotIdToClosestTilesMap) {
                this._localVoronoiRegions.Add(new VoronoiRegion(kv.Key, kv.Value));
            }

            return;
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
            if (_currentTarget.HasValue)
                return $"Voronoi current target: x:{_currentTarget.Value.x}, y:{_currentTarget.Value.y}";
            else return "Voronoi has no target";

        }
    }
}