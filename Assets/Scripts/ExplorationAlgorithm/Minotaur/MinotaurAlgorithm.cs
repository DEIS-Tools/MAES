using Maes.Map;
using Maes.Robot;
using System;
using System.Linq;
using System.Collections.Generic;
using System.Linq;
using UnityEditor.Graphs;
using UnityEngine;
using Maes.Utilities;
using static Maes.Map.SlamMap;
using UnityEngine.UIElements;

namespace Maes.ExplorationAlgorithm.Minotaur
{
    public partial class MinotaurAlgorithm : IExplorationAlgorithm
    {
        public float VisionRadius => _robotConstraints.SlamRayTraceRange;
        private IRobotController _controller;
        private RobotConstraints _robotConstraints;
        private CoarseGrainedMap _map;
        private Dictionary<Vector2Int, SlamTileStatus> _visibleTiles => _controller.GetSlamMap().GetCurrentlyVisibleTiles();
        private int _seed;
        private Vector2Int _location => _map.GetCurrentPositionCoarseTile();
        private List<Doorway> _doorways;
        private List<MinotaurAlgorithm> _minotaurs;
        private CardinalDirection.RelativeDirection _followDirection = CardinalDirection.RelativeDirection.Right;
        private State _currentState;
        private bool _taskBegun;
        private List<RelativeWall> _wallPoints = new();
        private Doorway _closestDoorway = null;

        private enum State
        {
            Idle,
            FirstWall,
            FollowingWall,
            ExploredAreaFound,
            StartRotation,
            Rotating,
            Auctioning,
            MovingToDoorway,
            MovingToNearestUnexplored
        }

        struct RelativeWall
        {
            public Vector2Int position;
            public float distance;
        }

        public MinotaurAlgorithm(RobotConstraints robotConstraints, int seed)
        {
            _robotConstraints = robotConstraints;
            _seed = seed;
        }

        public string GetDebugInfo()
        {
            return $"State: {Enum.GetName(typeof(State), _currentState)}" +
                   $"\nCoarse Map Position: {_map.GetApproximatePosition()}";
        }

        public void SetController(Robot2DController controller)
        {
            _controller = controller;
            _map = _controller.GetSlamMap().GetCoarseMap();
        }

        public void UpdateLogic()
        {
            if (_controller.HasCollidedSinceLastLogicTick())
            {
                return;
                //TODO: full resets
            }
            _wallPoints = GetWallsNearRobot();

            Vector2Int direction;
            switch (_currentState)
            {
                case State.Idle:
                    _controller.StartMoving();
                    _currentState = State.FirstWall;
                    break;
                case State.FirstWall:
                    if (_wallPoints.Any() && _wallPoints.First().distance < VisionRadius - 1)
                    {
                        _controller.StopCurrentTask();
                        _currentState = State.FollowingWall;
                    }
                    break;
                case State.FollowingWall:
                    if (_controller.GetStatus() == Robot.Task.RobotStatus.Idle)
                    {
                        var closestWall = _wallPoints.First();
                        CardinalDirection directionToWall = CardinalDirection.VectorToDirection(closestWall.position - _location);
                        direction = directionToWall.Counterclockwise().Vector;
                        var wallRejectorForce = new Vector2Int();

                        if (IsAheadExplored())
                        {
                            _currentState = State.ExploredAreaFound;
                            direction *= 2;
                        }

                        if (closestWall.distance < VisionRadius - 2)
                        {
                            wallRejectorForce = -directionToWall.Vector * (int)(VisionRadius - 2 - closestWall.distance);
                        }
                        _controller.MoveTo(_location + direction + wallRejectorForce);
                    }
                    break;
                case State.ExploredAreaFound:
                    direction = CardinalDirection.AngleToDirection(_controller.GetGlobalAngle()).Vector;
                    _controller.StartRotatingAroundPoint(_location + direction * (int)VisionRadius * 2);
                    break;
                case State.StartRotation:
                    _currentState = State.Rotating;
                    _wallPoints = GetWallsNearRobot();
                    _controller.Rotate(90);
                    break;
                case State.Rotating:
                    _controller.StartRotatingAroundPoint(_wallPoints.First().position);
                    _currentState = State.Idle;
                    break;
                case State.Auctioning:
                    break;
                case State.MovingToDoorway:
                    MoveToNearestUnexploredAreaWithinRoom();
                    break;
                case State.MovingToNearestUnexplored:
                    MoveThroughNearestUnexploredDoorway();
                    break;
                default:
                    break;
            }
        }

        private bool IsAheadExplored()
        {
            var direction = CardinalDirection.AngleToDirection(_controller.GetGlobalAngle());
            var target = direction.Vector * ((int)VisionRadius + 1) + _location;
            if (_map.IsWithinBounds(target))
            {
                return _map.GetTileStatus(target) == SlamTileStatus.Open;
            }
            return false;
        }

        private List<RelativeWall> GetWallsNearRobot()
        {
            return _visibleTiles.Where(kv => kv.Value == SlamTileStatus.Solid)
                                     .Select(kv => new RelativeWall { position = _map.FromSlamMapCoordinate(kv.Key), distance = Vector2.Distance(_map.FromSlamMapCoordinate(kv.Key), _location) })
                                     .OrderBy(dist => dist.distance)
                                     .ToList();
        }

        private void Communication()
        {
            var messages = _controller.ReceiveBroadcast().OfType<IMinotaurMessage>();
            var minotaurMessages = new List<IMinotaurMessage>();
            foreach (var message in messages)
            {
                var combined = false;
                foreach (var minotaurMessage in minotaurMessages)
                {
                    var combinedMessage = minotaurMessage.Combine(message, this);
                    if (combinedMessage != null)
                    {
                        minotaurMessages[minotaurMessages.IndexOf(message)] = combinedMessage;
                        combined = true;
                        break;
                    }
                }
                if (!combined)
                {
                    minotaurMessages.Add(message);
                }
            }
        }

        private void MoveToNearestUnexploredAreaWithinRoom()
        {
            //If guard clauses could be avoided, that would be great
            if (_closestDoorway == null)
            {
                int doorwayDistance = int.MaxValue;
                bool didNotPassDoorway = true;
                foreach (Doorway doorway in _doorways)
                {
                    List<Vector2Int> path = _controller.GetSlamMap().GetPath(_location, doorway.Position);
                    var distance = path.Count;
                    if (distance < doorwayDistance)
                    {
                        for (int i = 0; i < distance - 1; i++)
                        {
                            foreach (Doorway doorwayForPath in _doorways)
                            {
                                if (path[i] == doorwayForPath.Position) didNotPassDoorway = false;
                            }
                        }
                        if (didNotPassDoorway)
                        {
                            doorwayDistance = distance;
                            _closestDoorway = doorway;
                        }
                    };
                }
            }
            if (_closestDoorway == null) return;
            _controller.PathAndMoveTo(_closestDoorway.Position);
            if (_location == _closestDoorway.Position)
            {
                _currentState = State.Idle;
                _closestDoorway = null;
            }
        }

        private void MoveThroughNearestUnexploredDoorway()
        {
            //If guard clauses could be avoided, that would be great
            if (_closestDoorway == null)
            {
                int doorwayDistance = int.MaxValue;
                foreach (Doorway doorway in _doorways)
                {
                    List<Vector2Int> path = _controller.GetSlamMap().GetPath(_location, doorway.Position);
                    var distance = path.Count;
                    if (distance < doorwayDistance)
                    {
                        doorwayDistance = distance;
                        _closestDoorway = doorway;
                    }
                }
            }
            if (_closestDoorway == null) return;
            _controller.PathAndMoveTo(_closestDoorway.Position);
            if (_location == _closestDoorway.Position)
            {
                _currentState = State.Idle;
                _closestDoorway = null;
            }
        }
    }
}
