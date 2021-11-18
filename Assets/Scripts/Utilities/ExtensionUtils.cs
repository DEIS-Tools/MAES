using System.Collections.Generic;
using System.Diagnostics;
using Dora.Robot;
using UnityEngine;

namespace Dora.Utilities {
    public static class ExtensionUtils {
        /// <summary>
        /// Extension method for converting a SLAM tile to a TNF cell
        /// </summary>
        public static (Vector2Int, float) ToTnfCell(this KeyValuePair<Vector2Int, SlamMap.SlamTileStatus> tile) {
            return tile.Value switch
            {
                SlamMap.SlamTileStatus.Unseen => (tile.Key, 0.5f),
                _ => (tile.Key, 0f)
            };
        }

        public static float ToTnfCellValue(this SlamMap.SlamTileStatus status) {
            return status switch
            {
                SlamMap.SlamTileStatus.Unseen => .5f,
                _ => 0f
            };
        }
    }
}