using Maes.Map;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using Maes.Utilities;
using static Maes.Map.SlamMap;
using System;

namespace Maes.ExplorationAlgorithm.Minotaur
{
    internal class EdgeDetector
    {
        public EdgeState State => UpdateState();
        public bool isStuck => GetTilesAroundRobot(_edgeSize, _defaultLimitors).Where(tile => _map.GetTileStatus(tile) != SlamTileStatus.Open).Any();

        private CoarseGrainedMap _map;
        private int _edgeSize;
        private int _visionRange;
        private readonly List<SlamTileStatus> _defaultLimitors = new List<SlamTileStatus> { SlamTileStatus.Solid };
        private Vector2Int _robotPosition => _map.GetCurrentPositionCoarseTile();

        public enum EdgeState
        {
            Forward,
            ForwardLeft,
            ForwardRight,
        }

        public EdgeDetector(CoarseGrainedMap map, float visionRange)
        {
            _map = map;
            _edgeSize = (int)visionRange + 1;
            _visionRange = (int)visionRange;
        }

        private EdgeState UpdateState()
        {
            var tiles = GetTilesAroundRobot(_edgeSize, _defaultLimitors);
            var angle = _map.GetApproximateGlobalDegrees();
            return EdgeState.ForwardRight;
        }

        public Vector2Int? GetNearestUnseenTile()
        {
            var angles = Enumerable.Range(0, 360).ToList();
            var range = 1;
            while (angles.Any())
            {
                var unseenTiles = new List<Vector2Int>();
                var removedAngles = new List<int>();
                foreach (var angle in angles)
                {
                    var tile = GetFurthestTileAroundRobot(_map.GetApproximateGlobalDegrees() + angle, range, _defaultLimitors);

                    if (_map.GetTileStatus(tile) == SlamTileStatus.Solid)
                    {
                        removedAngles.Add(angle);
                        continue;
                    }
                    if (_map.GetTileStatus(tile) == SlamTileStatus.Unseen)
                    {
                        unseenTiles.Add(tile);
                    }
                }
                foreach (var removedangle in removedAngles)
                {
                    angles.Remove(removedangle);
                }
                if (unseenTiles.Any())
                {
                    return unseenTiles.OrderBy(tile => Vector2.Distance(_robotPosition, tile)).First();
                }
                range++;
            }
            return null;
        }

        public IEnumerable<Vector2Int> GetTilesAroundRobot(int range, List<SlamTileStatus> limiters, int startAngle = 0)
        {
            var tiles = new List<Vector2Int>();
            for (int angle = startAngle; angle < 360; angle++)
            {
                tiles.Add(GetFurthestTileAroundRobot(_map.GetApproximateGlobalDegrees() + angle, range, limiters));
            }
            return tiles.Distinct().Where(tile => _map.IsWithinBounds(tile));
        }

        public Vector2Int GetFurthestTileAroundRobot(float angle, int range, List<SlamTileStatus> limiters, bool snapToGrid = false)
        {
            Vector2Int tile = _robotPosition;
            for (int r = 0; r < range; r++)
            {
                tile = snapToGrid ? CardinalDirection.AngleToDirection(angle).Vector * r : Vector2Int.FloorToInt(Geometry.VectorFromDegreesAndMagnitude(angle, r));
                var candidateTile = tile + _robotPosition;
                if (_map.IsWithinBounds(candidateTile))
                {
                    foreach (var limiter in limiters)
                    {
                        if (_map.GetTileStatus(candidateTile) == limiter && (limiter != SlamTileStatus.Open || r > _visionRange))
                        {
                            return candidateTile;
                        }
                    }
                    tile = candidateTile;
                }
            }
            return tile;
        }

        public IEnumerable<Vector2Int> GetBoxAroundRobot()
        {
            var boxTileList = new List<Vector2Int>();
            for (int x = -_edgeSize; x <= _edgeSize; x++)
            {
                for (int y = _edgeSize; y <= _edgeSize+1; y++)
                {
                    boxTileList.Add(_robotPosition + new Vector2Int(x, y));
                    boxTileList.Add(_robotPosition + new Vector2Int(x, -y));
                    boxTileList.Add(_robotPosition + new Vector2Int(y,x));
                    boxTileList.Add(_robotPosition + new Vector2Int(y,-x));
                }
            }
            return boxTileList.Where(tile => _map.IsWithinBounds(tile));
        }


    }
}
