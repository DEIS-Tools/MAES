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

namespace Maes.ExplorationAlgorithm.Minotaur
{
    public partial class MinotaurAlgorithm : IExplorationAlgorithm
    {
        public float VisionArea => _robotConstraints.SlamRayTraceRange;
        private IRobotController _controller;
        private RobotConstraints _robotConstraints;
        private CoarseGrainedMap _map;
        private Dictionary<Vector2Int, SlamTileStatus> _visibleTiles => _controller.GetSlamMap().GetCurrentlyVisibleTiles();
        private int _seed;
        private Vector2Int _location => Vector2Int.RoundToInt(_map.GetApproximatePosition());
        private List<Doorway> _doorways;
        private List<MinotaurAlgorithm> _minotaurs;
        private CardinalDirection.RelativeDirection _followDirection = CardinalDirection.RelativeDirection.Right;
        private State _currentState;
        private bool _taskBegun;
        private RelativeWall? _wallPoint = null;
        private Doorway _closestDoorway = null;
        private enum State
        {
            Idle,
            FirstWall,
            Exploring,
            StartRotation,
            FinishedRotation,
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

            switch (_currentState)
            {
                case State.Idle:
                    _controller.StartMoving();
                    _currentState = State.FirstWall;
                    break;
                case State.FirstWall:
                    _wallPoint = GetWallNearRobot();
                    if (_wallPoint.HasValue)
                    {
                        _controller.StopCurrentTask();
                        _currentState = State.StartRotation;
                    }
                    break;
                case State.Exploring:
                    break;
                case State.StartRotation:
                    if (_controller.GetStatus() == Robot.Task.RobotStatus.Idle)
                    {
                        if (_taskBegun)
                        {
                            _currentState = State.Rotating;
                            _taskBegun = false;
                        }
                        else
                        {
                            _wallPoint = GetWallNearRobot();
                            _controller.Rotate(90);
                            _taskBegun = true;
                        }
                    }
                    break;
                case State.Rotating:
                    if (_controller.GetStatus() == Robot.Task.RobotStatus.Idle && _wallPoint.HasValue)
                    {
                        if (_taskBegun)
                        {
                            _currentState = State.Idle;
                            _taskBegun = false;
                        }
                        else
                        {
                            _controller.StartRotatingAroundPoint(_wallPoint.Value.position);
                            _taskBegun = true;
                        }
                    }
                    break;
                case State.Auctioning:
                    break;
                case State.MovingToDoorway:
                    MoveToNearestUnexploredAreaWithinRoom();
                    break;
                case State.MovingToNearestUnexplored:
                    MoveThroughNearestUnexploredDoorway();
                    break;
                case State.FinishedRotation:
                    break;
                default:
                    break;
            }
        }

        private RelativeWall? GetWallNearRobot()
        {
            var tiles = _visibleTiles.Where(kv => kv.Value == SlamTileStatus.Solid)
                                     .Select(kv => new RelativeWall { position = _map.FromSlamMapCoordinate(kv.Key), distance = Vector2.Distance(_map.FromSlamMapCoordinate(kv.Key), _location) })
                                     .OrderBy(dist => dist.distance);
            if (tiles.Any())
            {
                return tiles.First();
            }
            return null;
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

        private void ExploringAlongEdge()
        {
            _currentState = State.Exploring;

            GetNextExplorationTarget();
            //DoorwayDetection();
        }

        private void GetNextExplorationTarget()
        {
            //var tilesTowardFollowDirection = GetWallPositionAroundRobot(_followDirection);
            //tilesTowardFollowDirection.ForEach(tileStatus => Debug.Log(tileStatus));

            var direction = IsTileBlocked(_map.GetRelativeNeighbour(_followDirection));
        }

        /// <summary>
        /// Determines if it is impossible to move to the given tile when exploring
        /// </summary>
        /// <param name="tileCoord">Coordinate of the tile to check</param>
        /// <returns>
        /// True for blocked and false for open
        /// </returns>
        private bool IsTileBlocked(Vector2Int tileCoord)
        {
            return IsTileWall(tileCoord) || _map.IsTileExplored(tileCoord);
        }

        /// <summary>
        /// Check if tile coordinates are a wall
        /// </summary>
        /// <param name="tileCoord">Coordinate of the tile to check</param>
        /// <returns>
        /// True if solid else false
        /// </returns>
        private bool IsTileWall(Vector2Int tileCoord)
        {
            return _map.GetTileStatus(tileCoord) == SlamMap.SlamTileStatus.Solid;
        }

        private void DoorwayDetection()
        {
            throw new System.NotImplementedException();
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
