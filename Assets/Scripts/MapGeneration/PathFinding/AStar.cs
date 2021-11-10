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

        public List<Vector2Int>? GetOptimisticPath(Vector2Int startCoordinate, Vector2Int targetCoordinate,
            IPathFindingMap pathFindingMap) {
            return GetPath(startCoordinate, targetCoordinate, pathFindingMap, true);
        }

        public List<Vector2Int>? GetPath(Vector2Int startCoordinate, Vector2Int targetCoordinate, IPathFindingMap pathFindingMap, bool beOptimistic = false) {
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
                        // Don't evaluate the current tile as a neighbor
                        if(neighbourCoord.Equals(currentCoordinate)) continue;
                        // We can only traverse diagonally, if the corner and side next to diagonal tile are not blocked
                       if (IsDiagonal(currentCoordinate, neighbourCoord) 
                            && !CanTraverseDiagonally(currentCoordinate, neighbourCoord, pathFindingMap, beOptimistic)) {
                            // Debug.Log($"Could not traverse diagonally between {currentCoordinate} and {neighbourCoord}");
                            continue;
                        }
                        
                        var isSolid = beOptimistic
                            ? pathFindingMap.IsOptimisticSolid(neighbourCoord) // Unseen = open
                            : pathFindingMap.IsSolid(neighbourCoord); // Unseen = wall
                        // if(isSolid) Debug.Log($"Solid tile at: {neighbourCoord}");
                        if (!isSolid || neighbourCoord == targetCoordinate) {
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

                if (loopCount > 10000) {
                    throw new Exception("A* could not find path within 10000 loop runs");
                }

                loopCount++;
            }

            return null;
        }

        private bool IsDiagonal(Vector2Int currentTile, Vector2Int neighbourTile) {
            return currentTile.x != neighbourTile.x && currentTile.y != neighbourTile.y;
        }

        private bool CanTraverseDiagonally(Vector2Int currentTile, Vector2Int diagonalTile, IPathFindingMap pathFindingMap, bool beOptimistic) {
            // Top right. Right side and top must be open
            if (currentTile.x + 1 == diagonalTile.x && currentTile.y + 1 == diagonalTile.y) {
                var rightSideTile = new Vector2Int(currentTile.x + 1, currentTile.y);
                var topTile = new Vector2Int(currentTile.x, currentTile.y + 1);
                var rightSideOpen = beOptimistic ? !pathFindingMap.IsOptimisticSolid(rightSideTile) : !pathFindingMap.IsSolid(rightSideTile);
                var topOpen = beOptimistic ? !pathFindingMap.IsOptimisticSolid(topTile) : !pathFindingMap.IsSolid(topTile);
                return topOpen && rightSideOpen;
            }
            // Bottom right. Right side and bottom must be open
            else if (currentTile.x + 1 == diagonalTile.x && currentTile.y - 1 == diagonalTile.y) {
                var rightSideTile = new Vector2Int(currentTile.x + 1, currentTile.y);
                var bottomTile = new Vector2Int(currentTile.x, currentTile.y - 1);
                var rightSideOpen = beOptimistic ? !pathFindingMap.IsOptimisticSolid(rightSideTile) : !pathFindingMap.IsSolid(rightSideTile);
                var bottomOpen = beOptimistic ? !pathFindingMap.IsOptimisticSolid(bottomTile) : !pathFindingMap.IsSolid(bottomTile);
                return bottomOpen && rightSideOpen;
            }
            // Bottom left. Left side and bottom must be open
            else if (currentTile.x - 1 == diagonalTile.x && currentTile.y - 1 == diagonalTile.y) {
                var leftSideTile = new Vector2Int(currentTile.x - 1, currentTile.y);
                var bottomTile = new Vector2Int(currentTile.x, currentTile.y - 1);
                var leftSideOpen = beOptimistic ? !pathFindingMap.IsOptimisticSolid(leftSideTile) : !pathFindingMap.IsSolid(leftSideTile);
                var bottomOpen = beOptimistic ? !pathFindingMap.IsOptimisticSolid(bottomTile) : !pathFindingMap.IsSolid(bottomTile);
                return bottomOpen && leftSideOpen;
            }
            // Top left. Left side and top must be open.
            else if (currentTile.x - 1 == diagonalTile.x && currentTile.y + 1 == diagonalTile.y){
                var leftSideTile = new Vector2Int(currentTile.x - 1, currentTile.y);
                var topTile = new Vector2Int(currentTile.x, currentTile.y + 1);
                var leftSideOpen = beOptimistic ? !pathFindingMap.IsOptimisticSolid(leftSideTile) : !pathFindingMap.IsSolid(leftSideTile);
                var topOpen = beOptimistic ? !pathFindingMap.IsOptimisticSolid(topTile) : !pathFindingMap.IsSolid(topTile);
                return topOpen && leftSideOpen;
            }
            else {
                throw new Exception(
                    "Tried to evaluate if possible to traverse diagonally, but the target was not diagonal to currentTile");
            }
            
        }

        private float EuclideanHeuristic(Vector2Int from, Vector2Int to) {
            // var xDif = Math.Abs(from.x - to.x);
            // var yDif = Math.Abs(from.y - to.y);
            //
            // var minDif = Math.Min(xDif, yDif);
            // var maxDif = Math.Max(xDif, yDif);
            //
            // float heuristic = maxDif - minDif;
            // if (minDif > 0)
            //     heuristic += Mathf.Sqrt(2f * Mathf.Pow(minDif, 2f));

            // return heuristic;
            return Vector2Int.Distance(from,to);
        }
        
        
        public List<Vector2Int> GetIntersectingTiles(List<Vector2Int> path, float robotRadius) {
            throw new System.NotImplementedException();
        }

        
    }
}