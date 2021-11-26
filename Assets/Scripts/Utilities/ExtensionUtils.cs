using System.Collections.Generic;
using Maes.Map;
using Maes.Robot;
using UnityEngine;

namespace Maes.Utilities {
    public static class ExtensionUtils {
        /// <summary>
        /// Extension method for converting a SLAM tile to a TNF cell
        /// </summary>
        public static (Vector2Int, float) ToTnfCell(this KeyValuePair<Vector2Int, SlamMap.SlamTileStatus> tile) {
            return (tile.Key, tile.Value.ToTnfCellValue());
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