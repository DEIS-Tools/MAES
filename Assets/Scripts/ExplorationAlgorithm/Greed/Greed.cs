using Maes.Map;
using Maes.Robot;
using System;
using System.Linq;
using System.Collections.Generic;
using UnityEngine;
using Maes.Utilities;
using static Maes.Map.SlamMap;
using Maes.Map.PathFinding;

namespace Maes.ExplorationAlgorithm.Greed
{
    public partial class GreedAlgorithm : IExplorationAlgorithm
    {

        private IRobotController _controller;
        private CoarseGrainedMap _map;
        private Dictionary<Vector2Int, SlamTileStatus> _visibleTiles => _controller.GetSlamMap().GetCurrentlyVisibleTiles();
        private Vector2Int _position => _map.GetCurrentPosition();
        private AlgorithmState _currentState = AlgorithmState.Idle;

        private Waypoint? _waypoint;
        private int _logicTicks = 0;
        private int _ticksSinceHeartbeat;
        private int _deadlockTimer = 0;
        private Vector2Int _previousPosition;
        private Waypoint _previousWaypoint;

        private enum AlgorithmState
        {
            Idle,
            ExploreRoom,
            Done
        }

        private struct Waypoint
        {
            public Vector2Int Destination;
            public WaypointType Type;

            public enum WaypointType
            {
                Greed,
            }

            public Waypoint(Vector2Int destination, WaypointType type)
            {
                Destination = destination;
                Type = type;
            }

            public override bool Equals(object obj)
            {
                if (obj is Waypoint other)
                {
                    return Destination == other.Destination
                           && Type == other.Type;
                }
                return false;
            }
        }

        public GreedAlgorithm()
        {
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
            _previousPosition = _position;
        }

        public void UpdateLogic()
        {
            _logicTicks++;
            _ticksSinceHeartbeat++;
            if (_ticksSinceHeartbeat == 10)
            {
                var ownHeartbeat = new HeartbeatMessage(_controller.GetSlamMap());
                _ticksSinceHeartbeat = 0;
                _controller.Broadcast(ownHeartbeat);
            }
            var receivedHeartbeat = new Queue<HeartbeatMessage>(_controller.ReceiveBroadcast().OfType<HeartbeatMessage>());
            if (receivedHeartbeat.Count > 1)
            {
                var combinedMessage = receivedHeartbeat.Dequeue();
                foreach (var message in receivedHeartbeat)
                {
                    combinedMessage = combinedMessage.Combine(message);
                }
            }

            if (_controller.IsCurrentlyColliding())
            {
                if (_controller.GetStatus() != Robot.Task.RobotStatus.Idle)
                    _controller.StopCurrentTask();
                else
                {
                    var openTile = _map.GetNearestTileFloodFill(_position, SlamTileStatus.Open);
                    if (openTile.HasValue)
                        _controller.MoveTo(openTile.Value);
                    else
                        _controller.Move(1, true);
                }
                _waypoint = null;
                return;
                //TODO: full resets
            }

            if (_deadlockTimer >= 5)
            {
                var waypoint = _waypoint;
                if (MoveToNearestUnseen()) ;
                if (waypoint.HasValue && waypoint.Equals(_waypoint))
                {
                    MoveToNearestUnseen(new HashSet<Vector2Int> { waypoint.Value.Destination });
                }
                _deadlockTimer = 0;
            }

            if (_waypoint.HasValue)
            {
                var waypoint = _waypoint.Value;
                if (_map.GetPath(waypoint.Destination, false, false) == null)
                {
                    MoveToNearestUnseen(new() { waypoint.Destination });
                    waypoint = _waypoint.Value;
                }
                _controller.PathAndMoveTo(waypoint.Destination);

                if (IsDestinationReached())
                {
                    _waypoint = null;
                }
                else
                {
                    if (_logicTicks % 10 == 0)
                    {
                        if (_previousPosition == _position)
                            _deadlockTimer++;
                        else
                        {
                            _previousPosition = _position;
                            _deadlockTimer = 0;
                        }
                    }
                    return;
                }
            }

            switch (_currentState)
            {
                case AlgorithmState.Idle:
                    _controller.StartMoving();
                    _currentState = AlgorithmState.ExploreRoom;
                    break;
                case AlgorithmState.ExploreRoom:
                    if (_controller.GetStatus() == Robot.Task.RobotStatus.Idle)
                    {
                        if (MoveToNearestUnseen()) break;
                        else _currentState = AlgorithmState.Done;
                    }
                    break;
                case AlgorithmState.Done:
                    break;
                default:
                    break;
            }
            if (_logicTicks % 10 == 0)
            {
                if (_previousPosition == _position)
                    _deadlockTimer++;
                else
                    _deadlockTimer = 0;
            }
            if (_waypoint.HasValue)
                _previousWaypoint = _waypoint.Value;
            _previousPosition = _position;
        }

        private bool MoveToNearestUnseen(HashSet<Vector2Int> excludedTiles = null)
        {
            var startCoordinate = _position;
            if (_map.GetTileStatus(startCoordinate) == SlamTileStatus.Solid)
            {
                var NearestOpenTile = _map.GetNearestTileFloodFill(startCoordinate, SlamTileStatus.Open, excludedTiles);
                if (NearestOpenTile.HasValue)
                {
                    startCoordinate = NearestOpenTile.Value;
                }
            }
            var tile = _map.GetNearestTileFloodFill(startCoordinate, SlamTileStatus.Unseen, excludedTiles);
            if (tile.HasValue)
            {
                tile = _map.GetNearestTileFloodFill(tile.Value, SlamTileStatus.Open);
                if (tile.HasValue)
                {
                    _controller.PathAndMoveTo(tile.Value);
                    _waypoint = new Waypoint(tile.Value, Waypoint.WaypointType.Greed);
                    return true;
                }
            }
            return false;
        }


        private bool IsDestinationReached()
        {
            return _waypoint.HasValue && _map.GetTileCenterRelativePosition(_waypoint.Value.Destination).Distance < 0.5f;
        }
    }

    public class HeartbeatMessage
        {
            internal SlamMap map;

            public HeartbeatMessage(SlamMap map)
            {
                this.map = map;
            }

            public HeartbeatMessage Combine(HeartbeatMessage otherMessage)
            {
                if (otherMessage is HeartbeatMessage heartbeatMessage)
                {
                    List<SlamMap> maps = new() { heartbeatMessage.map, map };
                    SlamMap.Synchronize(maps); //layers of pass by reference, map in controller is updated with the info from message
                    return this;
                }
                return null;
            }

            public HeartbeatMessage Process() //Combine all, then process, but not really anything to process for heartbeat
            {
                return this;
            }
        }
}
