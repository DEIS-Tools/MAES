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
        private int _voronoiMinimumRange; // TODO assign dynamically according to constraints maybe?

        private enum ExplorationMode {
            SEARCH_MODE, EXPLORE_MODE
        }
        
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
            if(_shouldRecalculateRegions)
                RecalculateVoronoiRegions();
            
            // Find unexplored tiles
            // TODO: The voronoi region may be empty, if the robot is 1/4 the size of a slam tile and is located between tiles, but surrounded by other robots
            var myRegion = _localVoronoiRegions.First(k => k.RobotId == this._robotController.GetRobotID());
            var unexploredTiles = FindUnexploredTiles(myRegion);

            // If local region contains unexplored tiles
            // Move to them in ordered fashion (north, east, south and west
            // Else
            // Enter search mode
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

            float voronoiMapRange = Mathf.Min(_voronoiMinimumRange,nearbyRobots[0].Distance);

            Dictionary<int, List<Vector2>> robotIdToClosestTilesMap = new Dictionary<int, List<Vector2>>();

            
            for (int x = (int)(myPosition.x - voronoiMapRange); x < (int)(myPosition.x + voronoiMapRange); x++) {
                for (int y = (int)(myPosition.y - voronoiMapRange); y < (int)(myPosition.y + voronoiMapRange); y++) {
                    var tilePosition = new Vector2(x, y);
                    float bestDistance = Utilities.Geometry.DistanceBetween(myPosition, tilePosition);
                    int bestRobotId = this._robotController.GetRobotID();
                    foreach (var robot in nearbyRobots) {
                        var otherPosition = robot.GetRelativePosition(myPosition);
                        float distance = Utilities.Geometry.DistanceBetween(otherPosition, tilePosition);
                        if (distance < bestDistance) {
                            bestDistance = distance;
                            bestRobotId = robot.item;
                        }
                    }
                    robotIdToClosestTilesMap[bestRobotId].Add(tilePosition);
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
            return "";
        }
    }
}