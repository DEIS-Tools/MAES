using System.Collections.Generic;
using JetBrains.Annotations;
using UnityEngine;

namespace Dora.MapGeneration {
    public interface IPathFinder {

        public List<Vector2Int>? GetPath(Vector2Int startCoordinate, Vector2Int targetCoordinate, IPathFindingMap pathFindingMap, bool beOptimistic = false, bool acceptPartialPaths = false);
        
        public List<Vector2Int>? GetOptimisticPath(Vector2Int startCoordinate, Vector2Int targetCoordinate, IPathFindingMap pathFindingMap, bool acceptPartialPaths = false);

        public List<Vector2Int> GetIntersectingTiles(List<Vector2Int> path, float robotRadius);

    }
}