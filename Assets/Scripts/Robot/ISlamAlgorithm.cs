using System.Collections.Generic;
using Maes.Map;
using Maes.Map.PathFinding;
using UnityEngine;

namespace Maes.Robot {
    public interface ISlamAlgorithm {
        public Vector2 GetApproxPosition();

        public Dictionary<Vector2Int, SlamMap.SlamTileStatus> GetExploredTiles();
        
        public Dictionary<Vector2Int, SlamMap.SlamTileStatus> GetCurrentlyVisibleTiles();

        public Vector2Int GetCurrentPositionSlamTile();

        public SlamMap.SlamTileStatus GetStatusOfTile(Vector2Int tile);

        public float GetRobotAngleDeg();
        
        public List<Vector2Int> GetPath(Vector2Int slamTileFrom, Vector2Int slamTileTo, bool acceptPartialPaths = false);
        
        public List<Vector2Int> GetOptimisticPath(Vector2Int coarseTileFrom, Vector2Int coarseTileTo, bool acceptPartialPaths = false);

        public RelativePosition GetRelativeSlamPosition(Vector2Int slamTileTarget);

        public RelativePosition GetRelativePosition(Vector2Int target);
        
        public CoarseGrainedMap GetCoarseMap();

        public VisibleTilesCoarseMap GetVisibleTilesCoarseMap();
    }
}