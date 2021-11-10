using System.Collections.Generic;
using Dora.MapGeneration;
using Dora.MapGeneration.PathFinding;
using UnityEngine;
using static Dora.Robot.SlamMap;

namespace Dora.Robot {
    public interface SlamAlgorithmInterface {
        public Vector2 GetApproxPosition();

        public Dictionary<Vector2Int, SlamTileStatus> GetExploredTiles();
        
        public Dictionary<Vector2Int, SlamTileStatus> GetCurrentlyVisibleTiles();

        public Vector2Int GetCurrentPositionSlamTile();

        public SlamTileStatus GetStatusOfTile(Vector2Int tile);

        public float GetRobotAngleDeg();
        
        public List<Vector2Int> GetPath(Vector2Int slamTileFrom, Vector2Int slamTileTo);
        
        public List<Vector2Int> GetOptimisticPath(Vector2Int slamTileFrom, Vector2Int slamTileTo);

        public RelativePosition GetRelativeSlamPosition(Vector2Int slamTileTarget);

        public RelativePosition GetRelativePosition(Vector2Int target);
        
        public CoarseGrainedMap GetCoarseMap();
    }
}