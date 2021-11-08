using System.Collections.Generic;
using UnityEngine;
using static Dora.Robot.SlamMap;

namespace Dora.Robot {
    public interface SlamAlgorithmInterface {
        public Vector2 GetApproxPosition();

        public Dictionary<Vector2Int, SlamTileStatus> GetExploredTiles();
        
        public Dictionary<Vector2Int, SlamTileStatus> GetCurrentlyVisibleTiles();

        public Vector2Int GetCurrentPositionTile();

        public SlamTileStatus GetStatusOfTile(Vector2Int tile);

        public float getRobotAngleDeg();
    }
}