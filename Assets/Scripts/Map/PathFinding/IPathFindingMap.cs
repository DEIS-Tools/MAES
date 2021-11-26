using UnityEngine;

namespace Maes.Map.PathFinding {
    public interface IPathFindingMap {

        public bool IsSolid(Vector2Int coordinate);

        public bool IsOptimisticSolid(Vector2Int coordinate);

        public float CellSize();

    }
}