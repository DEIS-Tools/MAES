using Maes.Map;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using Maes.Utilities;
using static Maes.Map.SlamMap;
using System;
using Maes.Map.PathFinding;

namespace Maes.ExplorationAlgorithm.Minotaur
{
    internal class EdgeDetector
    {
        public EdgeState State => UpdateState();
        public bool isStuck => GetTilesAroundRobot(_edgeSize, _defaultLimitors).Where(tile => _coarseMap.GetTileStatus(tile) != SlamTileStatus.Open).Any();

        private SlamMap _slamMap;
        private CoarseGrainedMap _coarseMap;
        private int _edgeSize;
        private int _visionRange;
        private readonly List<SlamTileStatus> _defaultLimitors = new List<SlamTileStatus> { SlamTileStatus.Solid };
        private Vector2Int _robotPosition => _coarseMap.GetCurrentPosition();

        public enum EdgeState
        {
            Forward,
            ForwardLeft,
            ForwardRight,
        }

        public EdgeDetector(SlamMap map, float visionRange)
        {
            _slamMap = map;
            _coarseMap = map.GetCoarseMap();
            _edgeSize = (int)visionRange + 1;
            _visionRange = (int)visionRange;
        }

        private EdgeState UpdateState()
        {
            var tiles = GetTilesAroundRobot(_edgeSize, _defaultLimitors);
            var angle = _coarseMap.GetApproximateGlobalDegrees();
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
                    var tile = GetFurthestTileAroundRobot(_coarseMap.GetApproximateGlobalDegrees() + angle, range, _defaultLimitors);

                    if (_coarseMap.GetTileStatus(tile) == SlamTileStatus.Solid)
                    {
                        removedAngles.Add(angle);
                        continue;
                    }
                    if (_coarseMap.GetTileStatus(tile) == SlamTileStatus.Unseen)
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

        /// <summary>
        /// Gets the tiles around the robot by casting 360-<paramref name="startAngle"/> rays. These rays expand from the robot and out being stopped by the <paramref name="limiters"/>.
        /// <para></para>
        /// If only one ray is desired, consider <seealso cref="GetFurthestTileAroundRobot(float, int, List{SlamTileStatus}, bool, bool)"/>
        /// </summary>
        /// <param name="range">The distance of the ray</param>
        /// <param name="limiters">What tiles should stop the rays</param>
        /// <param name="slamPrecision">Target slam tiles instead of coarse tiles</param>
        /// <param name="startAngle">If set above 0 then this will create arcs instead of circles around the robot, based on <see cref="Vector2.right"/> counter-clockwise</param>
        /// <returns>The unique tiles that were hit</returns>
        public IEnumerable<Vector2Int> GetTilesAroundRobot(int range, List<SlamTileStatus> limiters, bool slamPrecision = false, int startAngle = 0)
        {
            IPathFindingMap map = slamPrecision ? _slamMap : _coarseMap;
            var tiles = new List<Vector2Int>();
            for (int angle = startAngle; angle < 360; angle++)
            {
                tiles.Add(GetFurthestTileAroundRobot(_coarseMap.GetApproximateGlobalDegrees() + angle, range, limiters, slamPrecision: slamPrecision));
            }
            return tiles.Distinct().Where(tile => map.IsWithinBounds(tile));
        }

        public Vector2Int GetFurthestTileAroundRobot(float angle, int range, List<SlamTileStatus> limiters, bool snapToGrid = false, bool slamPrecision = false)
        {
            var position = slamPrecision ? _slamMap.GetCurrentPosition() : _robotPosition;
            IPathFindingMap map = slamPrecision ? _slamMap : _coarseMap;
            Vector2Int tile = position;
            for (int r = 0; r < (slamPrecision ? range*2 : range); r++)
            {
                tile = snapToGrid ? CardinalDirection.AngleToDirection(angle).Vector * r : Vector2Int.FloorToInt(Geometry.VectorFromDegreesAndMagnitude(angle, r));
                var candidateTile = tile + position;
                if (map.IsWithinBounds(candidateTile))
                {
                    foreach (var limiter in limiters)
                    {
                        if (map.GetTileStatus(candidateTile) == limiter && (limiter != SlamTileStatus.Open || r > _visionRange))
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
                for (int y = _edgeSize; y <= _edgeSize + 1; y++)
                {
                    boxTileList.Add(_robotPosition + new Vector2Int(x, y));
                    boxTileList.Add(_robotPosition + new Vector2Int(x, -y));
                    boxTileList.Add(_robotPosition + new Vector2Int(y, x));
                    boxTileList.Add(_robotPosition + new Vector2Int(-y, x));
                }
            }
            return boxTileList.Where(tile => _coarseMap.IsWithinBounds(tile));
        }


    }
}
