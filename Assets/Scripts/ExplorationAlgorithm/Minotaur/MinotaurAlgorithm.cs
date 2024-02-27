using Maes.ExplorationAlgorithm.Minotaur.Messages;
using Maes.Map;
using Maes.Robot;
using System;
using System.Linq;
using System.Collections.Generic;
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
        private Vector2Int _location => _controller.GetSlamMap().GetCurrentPositionSlamTile();
        private List<Doorway> _doorways;
        private List<MinotaurAlgorithm> _minotaurs;
        private CardinalDirection.RelativeDirection _followDirection = CardinalDirection.RelativeDirection.Right;
        private State _currentState;
        private bool _taskBegun;
        private Vector2Int? _wallPoint = null;
        private enum State
        {
            Idle,
            Exploring,
            StartRotation,
            FinishedRotation,
            Rotating,
            Auctioning,
            MovingToDoorway,
            MovingToNearestUnexplored
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
                _currentState = State.Idle;
                //TODO: full resets
            }
            FindWall();


            //Communication();
            //ExploringAlongEdge();
            //MoveToNearestUnexploredAreaWithinRoom();
            //MoveThroughNearestUnexploredDoorway();
        }

        private void FindWall()
        {
            switch (_currentState)
            {
                case State.Idle:
                    _controller.StartMoving();
                    _currentState = State.Exploring;
                    break;
                case State.Exploring:
                    _wallPoint = GetWallNearRobot();
                    if (_wallPoint.HasValue)
                    {
                        _controller.StopCurrentTask();
                        _currentState = State.StartRotation;
                    }
                    break;
                case State.StartRotation: if (_controller.GetStatus() == Robot.Task.RobotStatus.Idle)
                    {
                        if (_taskBegun)
                        {
                            _currentState = State.Rotating;
                            _taskBegun = false;
                        }
                        else
                        {
                            _wallPoint = GetWallNearRobot();
                            //var robotToWallDirection = _location - _wallPoint.Value;
                            //var globalAngleToFace = Vector2.SignedAngle(Vector2.right, Vector2.Perpendicular((Vector2) robotToWallDirection));
                            //Debug.Log($"wallAngle: {globalAngleToFace}, robotAngle: {_controller.GetGlobalAngle()}");
                            //var rotateAngle = _controller.GetGlobalAngle() - globalAngleToFace;
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
                            _controller.StartRotateAroundPoint(_wallPoint.Value);
                            _taskBegun = true;
                        }
                    }
                    break;
                case State.Auctioning:
                    break;
                case State.MovingToDoorway:
                    break;
                case State.MovingToNearestUnexplored:
                    break;
                default:
                    break;
            }
        }

        private Vector2Int? GetWallNearRobot()
        {
            var local = _visibleTiles.OrderByDescending(dict => dict.Key.y).ToList();
            return new Vector2Int(105,98); // TODO: visibleTiles is never set :)
            foreach (var (position, tileStatus) in local)
            {
                if (tileStatus == SlamTileStatus.Solid)
                {
                }
            }
            return null;
        }

        private Vector2Int? GetWallPositionAroundRobot(CardinalDirection.RelativeDirection relativeDirection, bool checkExplored = true)
        {
            var range = (int)VisionArea;
            var directionVector = _map.GetRelativeNeighbourDirection(relativeDirection).Vector;
            var currentPosition = Vector2Int.FloorToInt(_map.GetApproximatePosition());

            if ((int)relativeDirection % 2 == 1)
                range = (int)MathF.Round(MathF.Sqrt(range));
            return Enumerable.Range(1, range)
                .Where(step => checkExplored ?
                                IsTileBlocked(currentPosition + directionVector * step) :
                                IsTileWall(currentPosition + directionVector * step))
                .Select(step => currentPosition + directionVector * step)
                .FirstOrDefault();
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
            var tilesTowardFollowDirection = GetWallPositionAroundRobot(_followDirection);
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
            throw new System.NotImplementedException();
        }

        private void MoveThroughNearestUnexploredDoorway()
        {
            throw new System.NotImplementedException();
        }
    }
}
