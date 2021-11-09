#nullable enable
using System;
using System.Collections.Generic;
using System.Linq;
using JetBrains.Annotations;
using UnityEngine;

namespace Dora.MapGeneration {
    public class AStar : IPathFinder {
        
        private class AStarTile : IComparable<AStarTile> {
            public readonly int X, Y;
            public AStarTile? Parent;
            public readonly float Heuristic;
            public readonly float Cost;
            public readonly float TotalCost;

            public AStarTile(int x, int y, AStarTile? parent, float heuristic, float cost) {
                X = x;
                Y = y;
                this.Parent = parent;
                Heuristic = heuristic;
                this.Cost = cost;
                this.TotalCost = cost + heuristic;
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

            public int CompareTo(AStarTile? other) {
                if (ReferenceEquals(this, other)) return 0;
                var thisTotalCost = this.Cost + this.Heuristic;
                var otherTotalCost = other.Cost + other.Heuristic;
                return thisTotalCost.CompareTo(otherTotalCost);
            }
        }

        public List<Vector2Int>? GetPath(Vector2Int startCoordinate, Vector2Int targetCoordinate, IPathFindingMap pathFindingMap) {
            var candidates = new SortedSet<AStarTile>(AStarTile.HeuristicComparer);
            var bestCandidateOnTile = new Dictionary<Vector2Int, AStarTile>();
            var startTileHeuristic = EuclideanHeuristic(startCoordinate, targetCoordinate);
            var startingTile = new AStarTile(startCoordinate.x, startCoordinate.y, null, startTileHeuristic, 0);
            candidates.Add(startingTile);
            bestCandidateOnTile[startCoordinate] = startingTile;

            int loopCount = 0; 
            while (candidates.Count > 0) {
                var currentTile = candidates.Min();
                candidates.Remove(currentTile);
                
                var currentCoordinate = new Vector2Int(currentTile.X, currentTile.Y);
                if (currentCoordinate == targetCoordinate)
                    return currentTile.Path();
                
                // Add all eight neighbours to the candidates list
                for (int x = currentTile.X - 1; x <= currentTile.X + 1; x++) {
                    for (int y = currentTile.Y - 1; y <= currentTile.Y + 1; y++) {
                        var neighbourCoord = new Vector2Int(x, y);
                        if (!pathFindingMap.IsSolid(neighbourCoord) || neighbourCoord == targetCoordinate) {
                            var cost = currentTile.Cost + Vector2Int.Distance(currentCoordinate, neighbourCoord);
                            var heuristic = EuclideanHeuristic(neighbourCoord, targetCoordinate);
                            var neighCost = cost + heuristic;
                            if (!bestCandidateOnTile.ContainsKey(neighbourCoord) || bestCandidateOnTile[neighbourCoord].TotalCost > neighCost) {
                                var newTile = new AStarTile(neighbourCoord.x, neighbourCoord.y, currentTile, heuristic,
                                    cost);
                                if(bestCandidateOnTile.ContainsKey(neighbourCoord))
                                    candidates.Remove(bestCandidateOnTile[neighbourCoord]);
                                bestCandidateOnTile[neighbourCoord] = newTile;
                                candidates.Add(newTile);
                            }

                        }
                    }
                }

                if (loopCount > 5000) {
                    throw new Exception("Fuck u");
                }

                loopCount++;
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