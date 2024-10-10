using System;
using Maes.Robot;
using UnityEngine;

namespace Maes.ExplorationAlgorithm.WallFollower {
    public partial class WallFollowerAlgorithm : IExplorationAlgorithm
    {
        private Robot2DController _controller;

        private bool _hasTurnedLeft = false;

        private float _gridSpacing = 2f;

        private float _angle;

        private bool _forwardWall;
        private bool _leftWall;

        private bool _rightWall;
        private bool _behindWall;

        private float _targetAngle => DirectionToAngle(_direction);

        private bool _collided = false;

        private bool _positionSet = false;
        private Vector2Int _targetPosition = new Vector2Int(2, 2);

        private const float NORTH = 90.0f;
        private const float EAST = 0.0f;
        private const float SOUTH = 270.0f;
        private const float WEST = 180.0f;

        private Direction _direction = Direction.North;

        //private int _beginAfter = 10;

        enum Direction {
            North,
            East,
            South,
            West,
            End,
        }


        public string GetDebugInfo()
        {
            return 
                $"Status: {_controller.GetStatus()}\n" +
                $"HasTurnedLeft: {_hasTurnedLeft}\n" +
                $"Walls: {(_forwardWall ? 'F' : '_')}{(_leftWall ? 'L' : '_')}{(_behindWall ? 'B' : '_')}{(_rightWall ? 'R' : '_')}\n" +
                $"Angle: {_angle} Target: {_targetAngle}\n"+
                $"Collided: {_collided}\n" +
                $"Pos: {_controller.GetSlamMap().GetCoarseMap().GetCurrentPosition()}\n" +
                $"TPos: {_targetPosition} Dir: {_direction}"
                ;
        }

        public void SetController(Robot2DController controller)
        {
            _controller = controller;
        }

        public void UpdateLogic()
        {
            _angle = _controller.GetGlobalAngle();

            if (!_positionSet) {
                //_targetPosition = _controller.GetSlamMap().GetCoarseMap().GetCurrentPosition();
                _positionSet = true;
            }

            if (_controller.IsRotating() || _controller.GetStatus() != Robot.Task.RobotStatus.Idle)
            {
                return;
            }
            

            _leftWall = IsWallLeft();
            _forwardWall = IsWallForward();
            _behindWall = IsWallBehind();
            _rightWall = IsWallRight();

            var epsilon = 0.5;

            if (_targetPosition != _controller.GetSlamMap().GetCoarseMap().GetCurrentPosition())
            {
                _controller.MoveTo(_targetPosition);
                return;
            }

            if (_angle > _targetAngle + epsilon || _angle < _targetAngle - epsilon) {
                _controller.Rotate(((_targetAngle - _angle) % 180.0f));
                return;
            }


            if (_hasTurnedLeft && (!_forwardWall && !_collided)) {
                GoForward();
                _collided = false;
                return;
            }

            if (!IsWallLeft()) {
                TurnLeft();
                _hasTurnedLeft = true;
                _collided = false;
                return;
            }

            if ((!_forwardWall && !_collided)) {
                GoForward();
                _collided = false;
                return;
            }

            TurnRight();
            //_hasTurnedLeft = false;
            _collided = false;
        }

        private float GetLeftGlobalAngle() {
            return ((_controller.GetGlobalAngle() + 90.0f) + 360.0f) % 360.0f;
        }

        private float GetRightGlobalAngle() {
            return (_controller.GetGlobalAngle() - 90.0f + 360.0f) % 360.0f;
        }

        private float GetForwardGlobalAngle() {
            return _controller.GetGlobalAngle();
        }

        private float DirectionToAngle(Direction direction) {
            switch (direction) {
                case Direction.North: return NORTH;
                case Direction.East: return EAST;
                case Direction.South: return SOUTH;
                case Direction.West: return WEST;
            }

            throw new InvalidOperationException($"INVALID DIRECTION: {direction}");
        }

        private Vector2Int DirectionToPosition(Direction direction) {
            switch (direction) {
                case Direction.North: return Vector2Int.up;
                case Direction.East: return Vector2Int.right;
                case Direction.South: return Vector2Int.down;
                case Direction.West: return Vector2Int.left;
            }

            throw new InvalidOperationException($"INVALID DIRECTION: {direction}");
        }

        private void TurnLeft() {
            Debug.Log("TurnLeft");

            _direction = (Direction)(mod((int)_direction - 1, (int)Direction.End));
        }

        private void TurnRight() {
            Debug.Log("TurnRight");

            _direction = (Direction)(mod((int)_direction + 1, (int)Direction.End));
        }

        private void GoForward() {
            Debug.Log("Forward");

            _targetPosition += DirectionToPosition(_direction);
            //_controller.MoveTo(_targetPosition);
        }

        private bool IsWallForward() {
            var wall = _controller.DetectWall(GetForwardGlobalAngle());
            if (!wall.HasValue)
            {
                return false;
            }
            return wall.Value.distance <= _gridSpacing;
        }

        private bool IsWallLeft() {
            var wall = _controller.DetectWall(GetLeftGlobalAngle());
            if (!wall.HasValue)
            {
                return false;
            }
            return wall.Value.distance <= _gridSpacing;
        }

        private bool IsWallRight() {
            var wall = _controller.DetectWall(GetRightGlobalAngle());
            if (!wall.HasValue)
            {
                return false;
            }
            return wall.Value.distance <= _gridSpacing;
        }

        private bool IsWallBehind() {
            var wall = _controller.DetectWall((GetForwardGlobalAngle() + 180.0f) % 360.0f);
            if (!wall.HasValue)
            {
                return false;
            }
            return wall.Value.distance <= _gridSpacing;
        }

        int mod(int x, int m) {
            return (x%m + m)%m;
        }
    }
}