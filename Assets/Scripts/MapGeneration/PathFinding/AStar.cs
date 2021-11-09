using System.Collections.Generic;
using UnityEngine;

namespace Dora.MapGeneration {
    public class AStar : IPathFinder {
        
        public List<Vector2Int> GetPath(Vector2Int startTile, Vector2Int targetTile, IPathFindingMap pathFindingMap) {
            throw new System.NotImplementedException();
        }
        public List<Vector2Int> GetIntersectingTiles(List<Vector2Int> path, float robotRadius) {
            throw new System.NotImplementedException();
        }
    }
}