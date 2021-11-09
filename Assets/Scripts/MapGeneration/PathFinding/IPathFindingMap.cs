using UnityEngine;

namespace Dora.MapGeneration {
    public interface IPathFindingMap {

        public bool IsSolid(Vector2Int coordinate);

        public float CellSize();

    }
}