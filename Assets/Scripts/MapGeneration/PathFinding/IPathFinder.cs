using System.Collections.Generic;
using UnityEngine;

namespace Dora.MapGeneration {
    public interface IPathFinder {

        public List<Vector2Int> GetPath(Vector2Int startingTile, Vector2Int targetTile, IPathFindingMap pathFindingMap);

        public List<Vector2Int> GetIntersectingTiles(List<Vector2Int> path, float robotRadius);

    }
}