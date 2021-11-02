using System;
using Dora.MapGeneration;
using Dora.Robot;
using UnityEngine;

namespace Dora.ExplorationAlgorithm {
    public class BrickAndMortar: IExplorationAlgorithm {

        private RobotConstraints _constraints;
        private IRobotController _controller;
        private int _randomSeed;

        // A safety measure to avoid being out of range due to inaccurate positioning by the robot
        private const float MaxExpectedPositioningError = 0.2f;
        
        // The distance from a tag, at which the robot must be able to read all 8 neighbouring tags
        private const float FullViewDistance = 0.2f;
        
        // The maximum diagonal distance between two tags
        private readonly float _maximumDiagonalDistance;
        // The maximum distance between along either axis (x or y)
        private readonly float _maximumAxialDistance;

        private readonly float _preferredAxialDistance;
        
        public BrickAndMortar(RobotConstraints constraints, int randomSeed) {
            _constraints = constraints;
            _randomSeed = randomSeed;
            
            _maximumDiagonalDistance = constraints.EnvironmentTagReadRange / 2f - MaxExpectedPositioningError;
            // Axial distance formula derived from a² + b² = c²  (And since we operate in a grid we have a = b)
            // This gives   2a² = c²  then rearranging to:  a = sqrt(c²/2)
            _maximumAxialDistance = Mathf.Sqrt(Mathf.Pow(_maximumAxialDistance, 2f) / 2f);
            
            _preferredAxialDistance = _maximumAxialDistance / 2f;
        }

        private bool test = false;
        public void UpdateLogic() {
            if (_controller.GetStatus() == RobotStatus.Idle) {
                if (test) {
                    _controller.Rotate(-42);
                    test = true;
                }
                else {
                    var result = _controller.DetectWall(90f);
                    if (result != null) {
                        var wall = result!.Value;
                        Debug.Log($"The wall is at distance: {wall.distance}, and relative angle: {wall.relativeAngle}");
                    }
                }
            }
            
            
        }

        public void SetController(Robot2DController controller) {
            this._controller = controller;
        }

        public string GetDebugInfo() {
            return "";
        }
        
        public object SaveState() {
            throw new System.NotImplementedException();
        }

        public void RestoreState(object stateInfo) {
            throw new System.NotImplementedException();
        }
    }
}