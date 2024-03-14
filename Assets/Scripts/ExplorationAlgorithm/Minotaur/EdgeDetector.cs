using Maes.Map;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using Maes.Utilities;
using static Maes.Map.SlamMap;

namespace Maes.ExplorationAlgorithm.Minotaur
{
    internal class EdgeDetector
    {
        public EdgeState State => UpdateState();
        public bool isStuck => GetTilesAroundRobot().Where(tile => _map.GetTileStatus(tile) != SlamTileStatus.Open).Any();

        private CoarseGrainedMap _map;
        private float _edgeSize;

        private Vector2Int _robotLocation => _map.GetCurrentPositionCoarseTile();
        private Vector2Int _unseenPoint => GetFurthestTileAroundRobot(_map.GetApproximateGlobalDegrees() - 45);
        private Vector2Int _solidOrExploredPoint => CardinalDirection.AngleToDirection(_map.GetApproximateGlobalDegrees() - 90).Vector + _unseenPoint;

        public enum EdgeState
        {
            Forward,
            ForwardLeft,
            ForwardRight,
        }

        public EdgeDetector(CoarseGrainedMap map, float edgeSize)
        {
            _map = map;
            _edgeSize = edgeSize;
        }

        public void DrawDebugLines()
        {

            var isPointUnseen = _map.GetTileStatus(_unseenPoint) == SlamTileStatus.Unseen;
            var isPointSolidOrExplored = _map.GetTileStatus(_solidOrExploredPoint) != SlamTileStatus.Unseen;
            var robot = _map.CoarseToWorld(_robotLocation);
            var point1 = _map.CoarseToWorld(_unseenPoint);
            var point2 = _map.CoarseToWorld(_solidOrExploredPoint);

            Debug.DrawLine(robot, point1, Color.magenta);
            Debug.DrawLine(robot, point2, Color.yellow);
            Debug.Log($"unseen: {isPointUnseen}, solid or explored: {isPointSolidOrExplored}");
        }

        private EdgeState UpdateState()
        {
            var tiles = GetTilesAroundRobot();
            var angle = _map.GetApproximateGlobalDegrees();
            return EdgeState.ForwardRight;
        }

        public IEnumerable<Vector2Int> GetTilesAroundRobot()
        {
            var tiles = new List<Vector2Int>();
            for (int angle = 0; angle < 360; angle++)
            {
                tiles.Add(GetFurthestTileAroundRobot(_map.GetApproximateGlobalDegrees() + angle));
            }
            return tiles.Distinct().Where(tile => _map.IsWithinBounds(tile));  //&& _map.GetTileStatus(tile) == SlamTileStatus.Unseen);
        }

        public Vector2Int GetFurthestTileAroundRobot(float angle)
        {
            Vector2Int tile = _robotLocation;
            for (int i = 0; i < _edgeSize; i++)
            {
                var candidateTile = Vector2Int.FloorToInt(Geometry.VectorFromDegreesAndMagnitude(angle, i)) + _robotLocation;
                if (_map.IsWithinBounds(candidateTile))
                {
                    if (_map.GetTileStatus(candidateTile) == SlamTileStatus.Solid)
                    {
                        return candidateTile;
                    }
                    tile = candidateTile;
                }
            }
            return tile;
        }

    }
}
