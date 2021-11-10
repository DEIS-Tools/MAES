using System;
using System.Collections.Generic;
using System.Linq;
using Dora.MapGeneration;
using Dora.MapGeneration.PathFinding;
using Dora.Robot;
using Dora.Utilities;
using UnityEngine;
using static Dora.MapGeneration.CardinalDirection;
using static Dora.MapGeneration.CardinalDirection.RelativeDirection;

namespace Dora.ExplorationAlgorithm.SSB {
    public class SsbAlgorithm : IExplorationAlgorithm {

        private IRobotController _controller;
        private RobotConstraints _constraints;
        private CoarseGrainedMap _navigationMap;
        private int _randomSeed;

        private State _currentState = State.Backtracking;
        private Vector2Int? _backtrackTarget;
        private Queue<Vector2Int>? _backtrackingPath;
        private Vector2Int? _nextBackTrackStep;

        // Spiraling information
        // The side that the outer wall of the spiral is on, relative to the spiraling robot
        private RelativeDirection _referenceLateralSide;
        // The opposite side of the rls. This is the side pointing toward the center of the spiral
        private RelativeDirection _oppositeLateralSide;
        // Target for next step in spiral movement
        private Vector2Int? _nextSpiralTarget;

        private HashSet<Vector2Int> _backTrackingPoints = new HashSet<Vector2Int>();
        
        private enum State {
            Spiraling,
            Backtracking,
            Terminated
        }

        private class TileData {
            public bool IsExplored = false;

            public TileData(bool isExplored) {
                IsExplored = isExplored;
            }
        }

        public SsbAlgorithm(RobotConstraints constraints, int randomSeed) {
            _constraints = constraints;
            _randomSeed = randomSeed;
            _backTrackingPoints.Add(new Vector2Int(22, 2));
        }

        public void UpdateLogic() {
            while (_controller.GetStatus() == RobotStatus.Idle && _currentState != State.Terminated) {
                if (_currentState == State.Backtracking) 
                    PerformBackTrack();
            
                if (_currentState == State.Spiraling)
                    PerformSpiraling();
            }
        }

        // --------------  Backtracking phase  -------------------
        private void PerformBackTrack() {
            _backtrackTarget ??= FindBestBackTrackingTarget();
            // If no backtracking targets exist, then exploration must have been completed
            if (_backtrackTarget == null) {
                _currentState = State.Terminated;
                return;
            }
            
            // Find path to target, if no path has been found previously
            _backtrackingPath ??= new Queue<Vector2Int>(_navigationMap.GetPath(_backtrackTarget!.Value));
            
            // Check if the path has been completed (ie. no more steps remain)
            if (_backtrackingPath.Count == 0) {
                // Robot has reached target, ensure that we are oriented at some angle
                // that is aligned with the grid before moving on to the spiral phase
                var isAligned = EnsureCorrectOrientationForSpiraling();
                if (isAligned) {
                    // Ready to enter Spiraling phase. Clear back tracking information
                    _currentState = State.Spiraling;
                    _nextBackTrackStep = null;
                    _backtrackingPath = null;
                }
                return;
            }
            
            _nextBackTrackStep ??= _backtrackingPath!.Dequeue();
            
            // Check if the robot needs to move to reach next step target 
            var relativeTarget = _navigationMap.GetTileCenterRelativePosition(_nextBackTrackStep!.Value);
            if (relativeTarget.Distance > 0.2f) 
                MoveTo(relativeTarget);
            else // We have reached this target, progress to next one
                _nextBackTrackStep = null;
        }

        // --------------  Spiraling phase  -------------------
        private void PerformSpiraling() {
            _nextSpiralTarget ??= DetermineNextSpiralTarget();

            if (_nextSpiralTarget == null) {
                _currentState = State.Backtracking;
                return;
            }
                
            var relativePosition = _navigationMap.GetTileCenterRelativePosition(_nextSpiralTarget!.Value);
            if (relativePosition.Distance > 0.3f) {
                MoveTo(relativePosition);
            } else {
                MarkTileExplored(_nextSpiralTarget!.Value);
                _nextSpiralTarget = null;
            }
        }

        private void MarkTileExplored(Vector2Int exploredTile) {
            _navigationMap.SetTileData(exploredTile, new TileData(true));
            // Remove this from possible back tracking candidates, if present
            _backTrackingPoints.Remove(exploredTile);
        }

        // Rotates the robot such that it directly faces a non-solid tile
        private bool EnsureCorrectOrientationForSpiraling() {
            CardinalDirection? targetDirection = null;
            var directions = new List<CardinalDirection>() {East, South, West, North};
            foreach (var direction in directions) {
                if (IsTileBlocked(_navigationMap.GetGlobalNeighbour(direction)))
                    continue;
                
                // If this tile is open and the left neighbour is blocked, start spiraling along the left wall 
                if (IsTileBlocked(_navigationMap.GetGlobalNeighbour(direction.GetRelativeDirection(Left)))) {
                    _referenceLateralSide = Left;
                    _oppositeLateralSide = Right;
                    targetDirection = direction;
                    break;
                }

                // If this tile is open and the right neighbour is blocked, start spiraling along the right wall
                if (IsTileBlocked(_navigationMap.GetGlobalNeighbour(direction.GetRelativeDirection(Right)))) {
                    _referenceLateralSide = Right;
                    _oppositeLateralSide = Left;
                    targetDirection = direction;
                    break;
                }
            }

            if (targetDirection == null)
                throw new NotImplementedException(); // TODO, Handle finish case?

            // Find relative position of neighbour located in target direction
            var targetRelativePosition = _navigationMap
                .GetTileCenterRelativePosition(_navigationMap.GetGlobalNeighbour(targetDirection));
            // Assert that we are pointed in the right direction
            if (Mathf.Abs(targetRelativePosition.RelativeAngle) <= 1.0f) 
                return true;
            
            // otherwise rotate to face the target tile
            _controller.Rotate(targetRelativePosition.RelativeAngle);
            return false;
        }

        private bool IsTileBlocked(Vector2Int tileCoord) {
            if (_navigationMap.GetTileStatus(tileCoord) == SlamMap.SlamTileStatus.Solid)
                return true; // physically blocked

            var tileData = _navigationMap.GetTileData(tileCoord);
            if (tileData != null) // If the tile is marked as explored, it is virtually blocked
                return ((TileData) tileData).IsExplored;

            // Neither physically nor virtually blocked
            return false;
        }

        private void MoveTo(RelativePosition relativePosition) {
            if (Mathf.Abs(relativePosition.RelativeAngle) > 0.5f)
                _controller.Rotate(relativePosition.RelativeAngle);
            else 
                _controller.Move(relativePosition.Distance);
        }

        private Vector2Int? FindBestBackTrackingTarget() {
            if (_backTrackingPoints.Count == 0) 
                return null;
            
            var robotPosition = _navigationMap.GetApproximatePosition();

            Vector2Int minDistanceBp = _backTrackingPoints.First();
            var minDist = Vector2.Distance(robotPosition, minDistanceBp);
            foreach (var bp in _backTrackingPoints.Skip(1)) {
                var dist = Vector2.Distance(robotPosition, bp);
                if (dist < minDist) {
                    minDist = dist;
                    minDistanceBp = bp;
                }
            }
            
            _backTrackingPoints.Remove(minDistanceBp);
            return minDistanceBp;
        }

        // Will perform spiral movement according to the algorithm described in the paper
        // Returns true if successful, and false if no free open are available 
        private Vector2Int? DetermineNextSpiralTarget() {
            var front = _navigationMap.GetRelativeNeighbour(Front);
            var rls = _navigationMap.GetRelativeNeighbour(_referenceLateralSide);
            var ols = _navigationMap.GetRelativeNeighbour(_oppositeLateralSide);
            
            var frontTileBlocked = IsTileBlocked(front);
            var rlsBlocked = IsTileBlocked(rls);
            var olsBlocked = IsTileBlocked(ols);
            
            // Add candidate tiles
            if (!rlsBlocked) _backTrackingPoints.Add(rls);
            if (!olsBlocked) _backTrackingPoints.Add(ols);
            if (!frontTileBlocked) _backTrackingPoints.Add(ols);
            
            if (frontTileBlocked && rlsBlocked && olsBlocked)
                return null; // No more open tiles left. Spiraling has finished

            // The following is the spiral algorithm specified by the bsa paper
            if (!rlsBlocked) 
                return _navigationMap.GetRelativeNeighbour(_referenceLateralSide);
            if (frontTileBlocked) 
                return _navigationMap.GetRelativeNeighbour(_oppositeLateralSide);
            return _navigationMap.GetRelativeNeighbour(Front);
        }
        
        public void SetController(Robot2DController controller) {
            this._controller = controller;
            _navigationMap = _controller.GetSlamMap().GetCoarseMap();
        }

        public string GetDebugInfo() {
            return $"State: {Enum.GetName(typeof(State), _currentState)}" +
                   $"\nCoarse Map Position: {_navigationMap.GetApproximatePosition()}";
        }

        public object SaveState() {
            throw new System.NotImplementedException();
        }

        public void RestoreState(object stateInfo) {
            throw new System.NotImplementedException();
        }
    }
}