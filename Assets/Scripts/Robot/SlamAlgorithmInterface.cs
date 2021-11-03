using System.Collections.Generic;
using UnityEngine;
using static Dora.Robot.SlamMap;

namespace Dora.Robot {
    public interface SlamAlgorithmInterface {
        public Vector2 GetApproxPosition();

        public List<(Vector2, SlamTileStatus)> GetExploredTiles();
    }
}