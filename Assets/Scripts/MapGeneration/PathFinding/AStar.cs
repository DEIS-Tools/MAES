#nullable enable
using System;
using System.Collections.Generic;
using System.Linq;
using JetBrains.Annotations;
using UnityEngine;

namespace Dora.MapGeneration {
    public class AStar : IPathFinder {
        
        private class AStarTile {
            public readonly int X, Y;
            public AStarTile? Parent;
            public readonly float Heuristic;
            public readonly float Cost;

            public AStarTile(int x, int y, AStarTile? parent, float heuristic, float cost) {
                X = x;
                Y = y;
                this.Parent = parent;
                Heuristic = heuristic;
                this.Cost = cost;
            }

            private sealed class HeuristicRelationalComparer : IComparer<AStarTile> {
                public int Compare(AStarTile x, AStarTile y) {
                    if (ReferenceEquals(x, y)) return 0;
                    var xTotalCost = x.Cost + x.Heuristic;
                    var yTotalCost = y.Cost + y.Heuristic;
                    return xTotalCost.CompareTo(yTotalCost);
                }
            }

            public static IComparer<AStarTile> HeuristicComparer { get; } = new HeuristicRelationalComparer();

            public List<Vector2Int> Path() {
                var path = new List<Vector2Int>();

                AStarTile? current = this;
                while (current != null) {
                    path.Add(new Vector2Int(current.X, current.Y));
                    current = current.Parent;
                }

                path.Reverse();
                return path;
            }
        }

        public List<Vector2Int>? GetPath(Vector2Int startTile, Vector2Int targetTile, IPathFindingMap pathFindingMap) {
            var candidates = new SortedSet<AStarTile>(AStarTile.HeuristicComparer);
            var startTileHeuristic = EuclideanHeuristic(startTile, targetTile);
            var startingTile = new AStarTile(startTile.x, startTile.y, null, startTileHeuristic, 0);
            candidates.Add(startingTile);

            while (candidates.Count > 0) {
                var currentTile = candidates.Min();
                candidates.Remove(currentTile);
                
                var currentCoordinate = new Vector2Int(currentTile.X, currentTile.Y);
                if (currentCoordinate == targetTile)
                    return currentTile.Path();
                
                // Add all eight neighbours to the candidates list
                for (int x = currentTile.X - 1; x <= currentTile.X + 1; x++) {
                    for (int y = currentTile.Y - 1; y <= currentTile.Y + 1; y++) {
                        var neighbourCoord = new Vector2Int(x, y);
                        var cost = currentTile.Cost + Vector2Int.Distance(currentCoordinate, neighbourCoord);
                        var heuristic = EuclideanHeuristic(neighbourCoord, targetTile);
                        candidates.Add(new AStarTile(neighbourCoord.x, neighbourCoord.y, currentTile, heuristic, cost));
                    }
                }
            }

            return null;
        }

        private float EuclideanHeuristic(Vector2Int from, Vector2Int to) {
            return Vector2Int.Distance(from, to);
        }
        
        
        public List<Vector2Int> GetIntersectingTiles(List<Vector2Int> path, float robotRadius) {
            throw new System.NotImplementedException();
        }
    }
}