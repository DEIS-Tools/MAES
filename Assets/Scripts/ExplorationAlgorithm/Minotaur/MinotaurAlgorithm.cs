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
        private EdgeDetector _edgeDetector;
        private Dictionary<Vector2Int, SlamTileStatus> _visibleTiles => _controller.GetSlamMap().GetCurrentlyVisibleTiles();
        private int _seed;
        private Vector2Int _robotPosition => _map.GetCurrentPositionCoarseTile();
        private List<Doorway> _doorways;
        private List<MinotaurAlgorithm> _minotaurs;
        private CardinalDirection.RelativeDirection _followDirection = CardinalDirection.RelativeDirection.Right;
        private AlgorithmState _currentState = AlgorithmState.Idle;
        private bool _taskBegun;
        private Doorway _closestDoorway = null;
        private Vector2Int? _destination;
        private RelativeWall _lastWall;

        private enum AlgorithmState
        {
            Idle,
            FirstWall,
            FollowingWall,
            FollowingExploredArea,
            StartRotation,
            Rotating,
            Auctioning,
            MovingToDoorway,
            MovingToNearestUnexplored
        }

        private struct RelativeWall
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
            return $"State: {Enum.GetName(typeof(AlgorithmState), _currentState)}" +
                   $"\nCoarse Map Position: {_map.GetApproximatePosition()}";
        }

        public void SetController(Robot2DController controller)
        {
            _controller = controller;
            _map = _controller.GetSlamMap().GetCoarseMap();
            _edgeDetector = new EdgeDetector(_map, VisionRadius);
        }

        public void UpdateLogic()
        {
            if (_controller.HasCollidedSinceLastLogicTick())
            {
                return;
                //TODO: full resets
            }
            var wallPoints = GetWallsNearRobot();

            Vector2Int direction;
            switch (_currentState)
            {
                case AlgorithmState.Idle:
                    _controller.StartMoving();
                    _currentState = AlgorithmState.FirstWall;
                    break;
                case AlgorithmState.FirstWall:
                    if (wallPoints.Any() && wallPoints.First().distance < VisionRadius - 1)
                    {
                        _controller.StopCurrentTask();
                        _currentState = AlgorithmState.FollowingWall;
                    }
                    break;
                case AlgorithmState.FollowingWall: // TODO: follow wall when no wall is found in visible area
                    if (_controller.GetStatus() == Robot.Task.RobotStatus.Idle)
                    {
                        var wallRejectorForce = new Vector2Int();
                        if (wallPoints.Any())
                        {
                            var closestWall = wallPoints.First();
                            CardinalDirection directionToWall = CardinalDirection.VectorToDirection(closestWall.position - _robotPosition);
                            direction = directionToWall.Counterclockwise().Vector;


                            if (closestWall.distance < VisionRadius - 2)
                            {
                                wallRejectorForce = -directionToWall.Vector * (int)(VisionRadius - 2 - closestWall.distance);
                            }
                            _lastWall = closestWall;
                        }
                        else
                        {
                            direction = CardinalDirection.VectorToDirection(_lastWall.position - _robotPosition).Vector;
                        }

                        if (IsAheadExplored())
                        {
                            _currentState = AlgorithmState.FollowingExploredArea;
                            direction *= 2;
                        }
                        _controller.MoveTo(_robotPosition + direction + wallRejectorForce);
                    }
                    break;
                case AlgorithmState.FollowingExploredArea:
                    if (_controller.GetStatus() == Robot.Task.RobotStatus.Idle)
                    {
                        if (_destination.HasValue)
                        {
                            _controller.MoveTo(_destination.Value);
                            ResetDestinationIfReached();
                            break;
                        }

                        if (MoveToCornerCoverage()) break;
                        else if (MoveToNearestEdge()) break;
                        else MoveToNearestUnseen();
                    }

                    break;
                case AlgorithmState.StartRotation:

                    if (_controller.GetStatus() == Robot.Task.RobotStatus.Idle)
                    {
                        _currentState = AlgorithmState.Rotating;
                        _controller.Rotate(90);
                    }
                    break;
                case AlgorithmState.Rotating:
                    if (_controller.GetStatus() == Robot.Task.RobotStatus.Idle)
                    {
                        _controller.StartRotatingAroundPoint(new Vector2Int(52, 54));
                    }
                    break;
                case AlgorithmState.Auctioning:
                    break;
                case AlgorithmState.MovingToDoorway:
                    MoveToNearestUnexploredAreaWithinRoom();
                    break;
                case AlgorithmState.MovingToNearestUnexplored:
                    MoveThroughNearestUnexploredDoorway();
                    break;
                default:
                    break;
            }
        }


        private bool MoveToCornerCoverage()
        {
            if (IsAheadExploreable(1) && IsAheadExploreable(2)) return false;
            var tiles = _edgeDetector.GetTilesAroundRobot((int)VisionRadius + 2, new List<SlamTileStatus> { SlamTileStatus.Unseen, SlamTileStatus.Solid }, 360 - 30);
            foreach (var tile in tiles)
            {
                tile.DrawDebugLineFromRobot(_map, Color.green);
            }
            // Take 30% of tiles to the right of the robots forwarding direction and filter them for unseen only
            var cornerCoverage = tiles.Where(tile => _map.GetTileStatus(tile) == SlamTileStatus.Unseen)
                                      .ToList();
            if (cornerCoverage.Any())
            {
                var location = cornerCoverage.First();
                foreach (var tile in cornerCoverage)
                {
                    tile.DrawDebugLineFromRobot(_map, Color.magenta);
                }
                _controller.MoveTo(location);
                _destination = location;
                return true;
            }
            return false;
        }
        private bool MoveToNearestEdge()
        {
            var tiles = _edgeDetector.GetBoxAroundRobot()
                               .Where(tile => _map.GetTileStatus(tile) != SlamTileStatus.Unseen
                                             && IsPerpendicularNotUnseen(tile))
                              .Select(tile => (status: _map.GetTileStatus(tile), perpendicularTile: tile + CardinalDirection.PerpendicularDirection(tile - _robotPosition).Vector * ((int)VisionRadius - 1)))
                              .Where(tile => _map.IsWithinBounds(tile.perpendicularTile) && IfSolidIsPathable(tile))
                              .Select(tile => tile.perpendicularTile);
            if (tiles.Any())
            {
                var closestTile = tiles.Select(tile => new { angle = _map.GetTileCenterRelativePosition(tile).RelativeAngle, tile }).OrderBy(tile => Mathf.Min(Math.Abs(tile.angle))).First().tile;
                var angle = Vector2.SignedAngle(Vector2.right, closestTile - _robotPosition);
                var vector = Geometry.VectorFromDegreesAndMagnitude(angle, VisionRadius + 1);
                Debug.Log($"tile: {closestTile}, angle: {angle}, vec: {vector}");
                closestTile = Vector2Int.FloorToInt(vector + _robotPosition);
                foreach (var tile in tiles)
                {
                    tile.DrawDebugLineFromRobot(_map, Color.yellow);
                }
                closestTile.DrawDebugLineFromRobot(_map, Color.white);
                _controller.MoveTo(closestTile);
                _destination = closestTile;
                return true;
            }
            return false;
        }

        private bool IsPerpendicularNotUnseen(Vector2Int tile)
        {
            var direction = CardinalDirection.PerpendicularDirection(tile - _robotPosition).Vector;
            return _map.GetTileStatus(tile + direction) == SlamTileStatus.Unseen;
        }
        private bool IfSolidIsPathable((SlamTileStatus status, Vector2Int perpendicularTile) tile)
        {
            if (tile.status != SlamTileStatus.Solid) return true;
            var path = _map.GetPath(tile.perpendicularTile, false, false);
            if (path == null)
            {
                return false;
            }
            return true;
        }

        private void MoveToNearestUnseen()
        {
            var tile = _edgeDetector.GetNearestUnseenTile();
            if (tile.HasValue)
            {
                _controller.PathAndMoveTo(tile.Value);
                _destination = tile;
                tile.Value.DrawDebugLineFromRobot(_map, Color.cyan);
            }
            else
            {
                _currentState = AlgorithmState.Auctioning;
            }
        }

        private void ResetDestinationIfReached()
        {
            if ((_destination.Value - _robotPosition).magnitude < 0.5f)
            {
                _destination = null;
            }

        }

        private bool IsAheadExplored()
        {
            var tiles = _edgeDetector.GetBoxAroundRobot();
            var direction = CardinalDirection.AngleToDirection(_controller.GetGlobalAngle());
            var target = direction.Vector * ((int)VisionRadius + 1) + _robotPosition;
            if (_map.IsWithinBounds(target))
            {
                return _map.GetTileStatus(target) == SlamTileStatus.Open;
            }
            return false;
        }

        private bool IsAheadExploreable(int range)
        {
            var direction = CardinalDirection.AngleToDirection(_controller.GetGlobalAngle());
            var target = direction.Vector * ((int)VisionRadius + range) + _robotPosition;
            if (_map.IsWithinBounds(target))
            {
                return _map.GetTileStatus(target) == SlamTileStatus.Unseen;
            }
            return false;
        }

        private List<RelativeWall> GetWallsNearRobot()
        {
            return _visibleTiles.Where(kv => kv.Value == SlamTileStatus.Solid)
                                     .Select(kv => new RelativeWall { position = _map.FromSlamMapCoordinate(kv.Key), distance = Vector2.Distance(_map.FromSlamMapCoordinate(kv.Key), _robotPosition) })
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
                    List<Vector2Int> path = _controller.GetSlamMap().GetPath(_robotPosition, doorway.Position);
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
            if (_robotPosition == _closestDoorway.Position)
            {
                _currentState = AlgorithmState.Idle;
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
                    List<Vector2Int> path = _controller.GetSlamMap().GetPath(_robotPosition, doorway.Position);
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
            if (_robotPosition == _closestDoorway.Position)
            {
                _currentState = AlgorithmState.Idle;
                _closestDoorway = null;
            }
        }
    }
}
