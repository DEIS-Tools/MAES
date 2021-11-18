using System.Collections.Generic;
using Dora.Robot;
using UnityEngine;

namespace Dora.MapGeneration.PathFinding {
    public class VisibleTilesCoarseMap : IPathFindingMap {
        private readonly SlamMap _slamMap;
        private readonly int _width;
        private readonly int _height;
        private readonly Vector2 _offset;
        private readonly AStar _aStar;


        public VisibleTilesCoarseMap(SlamMap slamMap, int width, int height, Vector2 offset) {
            _slamMap = slamMap;
            _width = width;
            _height = height;
            _offset = offset;
            _aStar = new AStar();
        }
        
        public List<Vector2Int>? GetPath(Vector2Int coarseTileFrom, Vector2Int coarseTileTo, bool acceptPartialPaths = false) {
            var path = _aStar.GetPath(coarseTileFrom, coarseTileTo, this, true, acceptPartialPaths);

            if (path == null)
                return null;

            // Due to rounding errors when converting slam tiles to path tiles, the target may not be correct
            // This replaces the final tile with the actual target.
            path[path.Count - 1] = coarseTileTo;

            return path;
        }

        public bool IsSolid(Vector2Int coordinate) {
            var slamTile = ToSlamMapCoordinate(coordinate);

            // Anything not currently visible is solid
            if (_slamMap._currentlyVisibleTiles[slamTile.x, slamTile.y] == SlamMap.SlamTileStatus.Unseen)
                return true;

            // Check if the coarse tile is actually solid
            var tileStatus = GetTileStatus(coordinate, false);
            if (tileStatus == SlamMap.SlamTileStatus.Solid)
                return true;

            return false;
        }

        public List<PathStep>? GetPathSteps(Vector2Int coarseTileFrom, Vector2Int coarseTileTo, bool acceptPartialPaths = false) {
            var path = _aStar.GetOptimisticPath(coarseTileFrom, coarseTileTo, this, acceptPartialPaths);
            return path == null ? null : _aStar.PathToSteps(path, 0.4f);
        }

        public bool IsOptimisticSolid(Vector2Int coordinate) {
            var slamTile = ToSlamMapCoordinate(coordinate);

            // Anything not currently visible is solid
            if (_slamMap._currentlyVisibleTiles[slamTile.x, slamTile.y] == SlamMap.SlamTileStatus.Unseen)
                return true;

            // Check if the coarse tile is actually solid
            var tileStatus = GetTileStatus(coordinate, true);
            if (tileStatus == SlamMap.SlamTileStatus.Solid)
                return true;

            return false;
        }

        public float CellSize() {
            return 1.0f; 
        }
        
        
        public Vector2Int FromSlamMapCoordinate(Vector2Int slamCoord) {
            return slamCoord / 2;
        }
        
        public Vector2Int ToSlamMapCoordinate(Vector2Int localCoordinate) {
            return localCoordinate * 2;
        }
        
        // Returns the status of the given tile (Solid, Open or Unseen)
        public SlamMap.SlamTileStatus GetTileStatus(Vector2Int localCoordinate, bool optismistic = false) {
            var slamCoord = ToSlamMapCoordinate(localCoordinate);

            var status = _slamMap.GetStatusOfTile(slamCoord);
            if (optismistic) {
                status = AggregateStatusOptimistic(status, _slamMap.GetStatusOfTile(slamCoord + Vector2Int.right));
                status = AggregateStatusOptimistic(status, _slamMap.GetStatusOfTile(slamCoord + Vector2Int.up));
                status = AggregateStatusOptimistic(status, _slamMap.GetStatusOfTile(slamCoord + Vector2Int.right + Vector2Int.up));    
            } else {
                status = AggregateStatusPessimistic(status, _slamMap.GetStatusOfTile(slamCoord + Vector2Int.right));
                status = AggregateStatusPessimistic(status, _slamMap.GetStatusOfTile(slamCoord + Vector2Int.up));
                status = AggregateStatusPessimistic(status, _slamMap.GetStatusOfTile(slamCoord + Vector2Int.right + Vector2Int.up));
            }
            
            return status;
        }
        
        // Combines two SlamTileStatus in a 'optimistic' fashion.
        // If any status is solid both are consider solid. Otherwise, if any status is open both are considered open
        // Unseen is returned only if all statuses are unseen 
        private SlamMap.SlamTileStatus AggregateStatusOptimistic(SlamMap.SlamTileStatus status1, SlamMap.SlamTileStatus status2) {
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
    }
}