using System;
using System.Collections.Generic;
using System.Linq;
using Dora.MapGeneration.PathFinding;
using Dora.Robot;
using UnityEngine;
using static Dora.MapGeneration.CardinalDirection;

namespace Dora.ExplorationAlgorithm.SSB {
    public class SsbAlgorithm : IExplorationAlgorithm {

        private IRobotController _controller;
        private RobotConstraints _constraints;
        private CoarseGrainedMap _navigationMap;
        private int _randomSeed;

        private State _currentState = State.Backtracking;
        private Vector2Int? _backtrackTarget;
        
        // Spiraling information
        // The side that the outer wall of the spiral is on, relative to the spiraling robot
        private RelativeDirection _referenceLateralSide;
        // The opposite side of the rls. This is the side pointing toward the center of the spiral
        private RelativeDirection _oppositeLateralSide;
        
        
        private HashSet<Vector2Int> _backTrackingPoints = new HashSet<Vector2Int>();
        
        private enum State {
            Spiraling,
            Backtracking,
            Terminated
        }

        public SsbAlgorithm(RobotConstraints constraints, int randomSeed) {
            _constraints = constraints;
            _randomSeed = randomSeed;
            _backTrackingPoints.Add(new Vector2Int(21, 2));
        }

        public void UpdateLogic() {
            if (_controller.GetStatus() != RobotStatus.Idle || _currentState == State.Terminated)
                return;

            if (_currentState == State.Backtracking) {
                _backtrackTarget ??= FindBestBackTrackingTarget();
                
                // If no backtracking targets exist, then exploration must have been completed
                if (_backtrackTarget == null) {
                    _currentState = State.Terminated;
                    return;
                }

                var relativeTarget = _navigationMap.GetTileCenterRelativePosition(_backtrackTarget!.Value);
                if (relativeTarget.Distance > 0.2f) 
                    MoveTo(relativeTarget);
                else { 
                    // Robot has reached target, ensure that we are oriented at some angle
                    // that is aligned with the grid before moving on to the spiral phase
                    var isAligned = EnsureCorrectOrientationForSpiraling();
                    if (isAligned) _currentState = State.Spiraling;
                }
            }
            
            if (_currentState == State.Spiraling) {
                var canContinueSpiral = PerformSpiralMovement();
                if (!canContinueSpiral)
                    _currentState = State.Backtracking;
            }
        }

        // Rotates the robot such that it directly faces a non-solid tile
        private bool EnsureCorrectOrientationForSpiraling() {
            var rotation = _navigationMap.GetApproximateGlobalDegrees();
            if (Mathf.Abs(rotation - 90f) <= 0.5f) 
                return true;
            
            _controller.Rotate(90f - rotation);
            return false;
        }

        private void MoveTo(RelativePosition relativePosition) {
            if (relativePosition.RelativeAngle > 0.5f)
                _controller.Rotate(relativePosition.RelativeAngle);
            else 
                _controller.Move(relativePosition.Distance);
        }

        private Vector2Int? FindBestBackTrackingTarget() {
            if (_backTrackingPoints.Count == 0) {
                return null;
            }

            var bestCandidate = _backTrackingPoints.First();
            _backTrackingPoints.Remove(bestCandidate);
            return bestCandidate;
        }

        // Will perform spiral movement according to the algorithm described in the paper
        // Returns true if successful, and false if no free open are available 
        private bool PerformSpiralMovement() {
            
            return false;
        }
        
        public void SetController(Robot2DController controller) {
            this._controller = controller;
            _navigationMap = _controller.GetSlamMap().GetCoarseMap();
        }

        public string GetDebugInfo() {
            return $"State: {Enum.GetName(typeof(State), _currentState)}";
        }

        public object SaveState() {
            throw new System.NotImplementedException();
        }

        public void RestoreState(object stateInfo) {
            throw new System.NotImplementedException();
        }
    }
}