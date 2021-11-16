using System.Collections.Generic;
using Dora.MapGeneration.PathFinding;
using JetBrains.Annotations;
using UnityEngine;

namespace Dora.MapGeneration {
    public interface IPathFinder {

        public List<Vector2Int>? GetPath(Vector2Int startCoordinate, Vector2Int targetCoordinate, IPathFindingMap pathFindingMap, bool beOptimistic = false);
        
        public List<Vector2Int>? GetOptimisticPath(Vector2Int startCoordinate, Vector2Int targetCoordinate, IPathFindingMap pathFindingMap);

        public List<PathStep> GetIntersectingTiles(List<Vector2Int> path, float robotRadius);

    }
}