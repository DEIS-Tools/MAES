#nullable enable
using System;
using System.Collections.Generic;
using System.Linq;
using Maes.Utilities;
using Maes.Utilities.Priority_Queue;
using UnityEngine;

namespace Maes.Map.PathFinding {
    public class AStar : IPathFinder {
        
        private class AStarTile {
            public readonly int X, Y;
            public AStarTile? Parent;
            public readonly float Heuristic;
            public readonly float Cost;
            public readonly float TotalCost;
            public readonly int TotalCostInt;

            public AStarTile(int x, int y, AStarTile? parent, float heuristic, float cost) {
                X = x;
                Y = y;
                this.Parent = parent;
                Heuristic = heuristic;
                this.Cost = cost;
                this.TotalCost = cost + heuristic;
                this.TotalCostInt = (int) (TotalCost * 1000f);
            }

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

        public List<Vector2Int>? GetOptimisticPath(Vector2Int startCoordinate, Vector2Int targetCoordinate, IPathFindingMap pathFindingMap, bool acceptPartialPaths = false) {
            return GetPath(startCoordinate, targetCoordinate, pathFindingMap, true, acceptPartialPaths);
        }

        public List<Vector2Int>? GetPath(Vector2Int startCoordinate, Vector2Int targetCoordinate, IPathFindingMap pathFindingMap, bool beOptimistic = false, bool acceptPartialPaths = false) {
            IPriorityQueue<AStarTile, float> candidates = new SimplePriorityQueue<AStarTile, float>();
            var bestCandidateOnTile = new Dictionary<Vector2Int, AStarTile>();
            var startTileHeuristic = OctileHeuristic(startCoordinate, targetCoordinate);
            var startingTile = new AStarTile(startCoordinate.x, startCoordinate.y, null, startTileHeuristic, 0);
            candidates.Enqueue(startingTile, startingTile.TotalCost);
            bestCandidateOnTile[startCoordinate] = startingTile;

            int loopCount = 0; 
            while (candidates.Count > 0) {
                
                var currentTile = candidates.Dequeue();
                var currentCoordinate = new Vector2Int(currentTile.X, currentTile.Y);
                
                // Skip if a better candidate has been added to the queue since this was added 
                if(bestCandidateOnTile.ContainsKey(currentCoordinate) && bestCandidateOnTile[currentCoordinate] != currentTile)
                    continue;

                if (currentCoordinate == targetCoordinate) 
                    return currentTile.Path();
                

                foreach (var dir in CardinalDirection.AllDirections()) {
                    Vector2Int candidateCoord = currentCoordinate + dir.Vector;
                    // Only consider non-solid tiles
                    if (IsSolid(candidateCoord, pathFindingMap, beOptimistic) && candidateCoord != targetCoordinate) continue;

                    if (dir.IsDiagonal()) {
                        // To travel diagonally, the two neighbouring tiles must also be free
                        if (IsSolid(currentCoordinate + dir.Previous().Vector, pathFindingMap, beOptimistic)
                        || IsSolid(currentCoordinate + dir.Next().Vector, pathFindingMap, beOptimistic))
                            continue;
                    }
                    
                    var cost = currentTile.Cost + Vector2Int.Distance(currentCoordinate, candidateCoord);
                    var heuristic = OctileHeuristic(candidateCoord, targetCoordinate);
                    var candidateCost = cost + heuristic;
                    // Check if this path is 'cheaper' than any previous path to this candidate tile 
                    if (!bestCandidateOnTile.ContainsKey(candidateCoord) || bestCandidateOnTile[candidateCoord].TotalCost > candidateCost) {
                        var newTile = new AStarTile(candidateCoord.x, candidateCoord.y, currentTile, heuristic, cost);
                        // Save this as the new best candidate for this tile
                        bestCandidateOnTile[candidateCoord] = newTile;
                        candidates.Enqueue(newTile, newTile.TotalCost);
                    }
                }

                if (loopCount > 100000) {
                    Debug.Log($"A star loop count exceeded 100000, stopping pathfinding prematurely. [{startCoordinate} -> {targetCoordinate}]");
                    return null;
                } 
                    

                loopCount++;
            }
            
            if (acceptPartialPaths) {
                // Find lowest heuristic tile, as it is closest to the target
                Vector2Int? lowestHeuristicKey = null;
                float lowestHeuristic = float.MaxValue;
                foreach (var kv in bestCandidateOnTile) {
                    if (kv.Value.Heuristic < lowestHeuristic) {
                        lowestHeuristic = kv.Value.Heuristic;
                        lowestHeuristicKey = kv.Key;
                    }
                }

                var closestTile = bestCandidateOnTile[lowestHeuristicKey.Value];
                return GetPath(startCoordinate, new Vector2Int(closestTile.X, closestTile.Y),
                    pathFindingMap, beOptimistic, false);
            }

            return null;
        }

        private AStarTile DequeueBestCandidate(List<AStarTile> candidates) {
            AStarTile bestCandidate = candidates.First();
            foreach (var current in candidates.Skip(1)) {
                if (Mathf.Abs(current.TotalCost - bestCandidate.TotalCost) < 0.01f) {
                    // Total cost is the same, compare by heuristic instead
                    if (current.Heuristic < bestCandidate.Heuristic)
                        bestCandidate = current;
                } else if (current.TotalCost < bestCandidate.TotalCost) {
                    bestCandidate = current;
                }
            }

            candidates.Remove(bestCandidate);
            return bestCandidate;
        }

        private bool IsSolid(Vector2Int coord, IPathFindingMap map, bool optimistic) {
            return optimistic
                ? map.IsOptimisticSolid(coord) 
                : map.IsSolid(coord); 
        }

        private static float OctileHeuristic(Vector2Int from, Vector2Int to) {
            var xDif = Math.Abs(from.x - to.x);
            var yDif = Math.Abs(from.y - to.y);
            
            var minDif = Math.Min(xDif, yDif);
            var maxDif = Math.Max(xDif, yDif);
            
            float heuristic = maxDif - minDif + minDif * Mathf.Sqrt(2f);
            return heuristic;
        }
        
        // Converts the given A* path to PathSteps (containing a line and a list of all tiles intersected in this path)
        public List<PathStep> PathToSteps(List<Vector2Int> path, float robotRadius) {
            if (path.Count == 1) 
                return new List<PathStep> { new PathStep(path[0], path[0], new HashSet<Vector2Int>(){path[0]})};
            var steps = new List<PathStep>();

            Vector2Int stepStart = path[0];
            Vector2Int currentTile = path[1];
            HashSet<Vector2Int> crossedTiles = new HashSet<Vector2Int>();
            CardinalDirection currentDirection = CardinalDirection.FromVector(currentTile - stepStart);
            AddIntersectingTiles(stepStart, currentDirection, crossedTiles);
            
            foreach (var nextTile in path.Skip(2)) {
                var newDirection = CardinalDirection.FromVector(nextTile - currentTile);
                if (newDirection != currentDirection) {
                    // New path step reached
                    steps.Add(new PathStep(stepStart, currentTile, crossedTiles));
                    crossedTiles = new HashSet<Vector2Int>();
                    stepStart = currentTile;
                    currentDirection = newDirection;
                } 
                AddIntersectingTiles(currentTile, currentDirection, crossedTiles);
                currentTile = nextTile;
            }
            
            steps.Add(new PathStep(stepStart, currentTile, crossedTiles));
            return steps;
        }

        public void AddIntersectingTiles(Vector2Int from, CardinalDirection direction, HashSet<Vector2Int> tiles) {
            tiles.Add(from);
            tiles.Add(from + direction.Vector);
            if (direction.IsDiagonal()) {
                // Two of the neighbouring tiles are also intersected when traversing tiles diagonally 
                tiles.Add(from + direction.Next().Vector);
                tiles.Add(from + direction.Previous().Vector);
            }
        }


    }
}