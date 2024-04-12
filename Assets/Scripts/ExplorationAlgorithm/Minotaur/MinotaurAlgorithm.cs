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
        public int VisionRadius => (int)_robotConstraints.SlamRayTraceRange;
        private int _doorWidth;

        private IRobotController _controller;
        private RobotConstraints _robotConstraints;
        private CoarseGrainedMap _map;
        private EdgeDetector _edgeDetector;
        private Dictionary<Vector2Int, SlamTileStatus> _visibleTiles => _controller.GetSlamMap().GetCurrentlyVisibleTiles();
        private int _seed;
        private Vector2Int _position => _map.GetCurrentPositionCoarseTile();
        private List<Doorway> _doorways = new();
        private List<MinotaurAlgorithm> _minotaurs;
        private bool _clockwise = false;
        private AlgorithmState _currentState = AlgorithmState.Idle;
        private DoorState _doorState = DoorState.None;
        private bool _taskBegun;
        private Doorway _closestDoorway = null;
        private Waypoint? _waypoint;
        private Vector2Int? _lastWallTile;
        private List<Line2D> _lastWalls = new();
        private int _logicTicks = 0;

        private enum AlgorithmState
        {
            Idle,
            FirstWall,
            ExploreRoom,
            Auctioning,
            MovingToDoorway,
            MovingToNearestUnexplored
        }
        private enum DoorState
        {
            None,
            Single,
            Intersection
        }

        private struct Waypoint
        {
            public Vector2Int Destination;
            public bool UsePathing;
            public WaypointType Type;

            public enum WaypointType
            {
                Wall,
                Corner,
                Edge,
                Greed,
                Door
            }

            public Waypoint(Vector2Int destination, WaypointType type, bool pathing = false)
            {
                Destination = destination;
                Type = type;
                UsePathing = pathing;
            }
        }

        private struct RelativeWall
        {
            public Vector2Int Position;
            public float Distance;
            public float Angle;
        }

        public MinotaurAlgorithm(RobotConstraints robotConstraints, int seed, int doorWidth)
        {
            _robotConstraints = robotConstraints;
            _seed = seed;
            _doorWidth = doorWidth;
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
            _logicTicks++;
            if (_controller.HasCollidedSinceLastLogicTick())
            {
                _controller.Move(1, true);
                _waypoint = null;
                return;
                //TODO: full resets
            }

            var wallPoints = GetWallsNearRobot();

            switch (_doorState)
            {
                case DoorState.None:
                    if (!_waypoint.HasValue || _waypoint.Value.Type == Waypoint.WaypointType.Wall)
                        _doorState = DoorwayDetection(wallPoints);
                    if (_doorState != DoorState.None)
                        _lastWalls = GetWalls(wallPoints.Select(tile => tile.Position));
                    break;
                case DoorState.Single:
                    if (IsDestinationReached())
                    {
                        _doorState = DoorState.None;
                        if (!wallPoints.Any()) break;
                        var walls = GetWalls(wallPoints.Select(tile => tile.Position));
                        walls.ToList().ForEach(wall => Debug.DrawLine(_map.CoarseToWorld(wall.Start), _map.CoarseToWorld(wall.End)));
                        if (walls.Any(wall => _lastWalls.Contains(wall)))
                        {
                            var (start, end) = SortSingleWall(walls);
                            var wallDirectionVector = CardinalDirection.DirectionFromDegrees((end - start).GetAngleRelativeToX()).Vector;
                            var closestWall = wallPoints.First();
                            var approachDirection = _clockwise ? CardinalDirection.DirectionFromDegrees((_controller.GetGlobalAngle() + 90) % 360) : CardinalDirection.DirectionFromDegrees((_controller.GetGlobalAngle() + 270) % 360);
                            _closestDoorway = new Doorway(_lastWallTile.Value, closestWall.Position, approachDirection);
                            _doorways.Add(_closestDoorway);
                        }
                    }
                    break;
                case DoorState.Intersection:
                    break;
                default:
                    break;
            }

            if (_waypoint.HasValue)
            {
                var waypoint = _waypoint.Value;

                if (waypoint.UsePathing)
                {
                    _controller.PathAndMoveTo(waypoint.Destination);
                }
                else
                {
                    var solidTile = _edgeDetector.GetFurthestTileAroundRobot((waypoint.Destination - _position).GetAngleRelativeToX(), VisionRadius, new List<SlamTileStatus> { SlamTileStatus.Solid }, true);
                    if (_map.GetTileStatus(solidTile) == SlamTileStatus.Solid)
                    {
                        _waypoint = null;
                        _controller.StopCurrentTask();
                        return;
                    }
                    _controller.MoveTo(waypoint.Destination);
                }
                if (IsDestinationReached())
                {
                    _waypoint = null;
                }
                return;
            }



            switch (_currentState)
            {
                case AlgorithmState.Idle:
                    _controller.StartMoving();
                    _currentState = AlgorithmState.FirstWall;
                    break;
                case AlgorithmState.FirstWall:
                    if (wallPoints.Any() && wallPoints.First().Distance < VisionRadius - 1)
                    {
                        _controller.StopCurrentTask();
                        _currentState = AlgorithmState.ExploreRoom;
                    }
                    break;
                case AlgorithmState.ExploreRoom:
                    if (_controller.GetStatus() == Robot.Task.RobotStatus.Idle)
                    {
                        if (!IsAroundExplorable(1))
                        {
                            MoveToNearestUnseen();
                            break;
                        }
                        else if (MoveAlongWall()) break;
                        else if (MoveToCornerCoverage()) break;
                        else if (MoveToNearestEdge()) break;
                        else MoveToNearestUnseen();
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
        private bool MoveAlongWall()
        {
            // Wait until SlamMap has updated otherwise we see old data
            if (_logicTicks % _robotConstraints.SlamUpdateIntervalInTicks != 0)
            {
                return true;
            }
            var tiles = _edgeDetector.GetTilesAroundRobot(VisionRadius + 2, new List<SlamTileStatus> { SlamTileStatus.Solid }).Where(tile => _map.GetTileStatus(tile) == SlamTileStatus.Solid).ToList();
            if (tiles.Count() < 2) return false;
            var tileAhead = _edgeDetector.GetFurthestTileAroundRobot(_controller.GetGlobalAngle(), VisionRadius, new List<SlamTileStatus> { SlamTileStatus.Solid }, true);
            var localLeft = (_controller.GetGlobalAngle() + (_map.GetTileStatus(tileAhead) == SlamTileStatus.Solid ? 90 : 0)) % 360;
            (CardinalDirection.AngleToDirection(localLeft).Vector + _position).DrawDebugLineFromRobot(_map, Color.blue);
            var walls = GetWalls(tiles);
            var wallPoints = walls.Select(wall => Vector2Int.FloorToInt(wall.Start))
                              .Union(walls.Select(wall => Vector2Int.FloorToInt(wall.End)))
                              .OrderByDescending(point => ((point - _position).GetAngleRelativeToX() - localLeft + 360) % 360).ToList();

            walls.ToList().ForEach(wall => Debug.DrawLine(_map.CoarseToWorld(wall.Start), _map.CoarseToWorld(wall.End), Color.black, 2));
            wallPoints.ToList().ForEach(point => point.DrawDebugLineFromRobot(_map, Color.yellow));
            var points = wallPoints.Select(point => (perp: point + CardinalDirection.PerpendicularDirection(point - _position).Vector * (VisionRadius - 2), point))
                           .Where(tuple => _map.IsWithinBounds(tuple.perp)
                                           && _map.GetTileStatus(tuple.perp) != SlamTileStatus.Solid)
                           .ToList();

            if (!points.Any())
            {
                return false;
            }


            var perpendicularTile = points.First();
            //var thirdPoint = new Vector2Int((int)(perpendicularTile.perp.x + (Math.Abs(perpendicularTile.perp.x - perpendicularTile.point.x))*(Vector2.Angle(_position,perpendicularTile.perp)/90)), (int)(perpendicularTile.perp.y + (Math.Abs(perpendicularTile.perp.y - perpendicularTile.point.y))*(1-Vector2.Angle(_position,perpendicularTile.perp)/90)));

            var perpDirection = (perpendicularTile.perp - _position).GetAngleRelativeToX();
            var thirdPointDirection = (perpDirection + 270) % 360;
            var thirdPointDirectionVector = CardinalDirection.AngleToDirection(thirdPointDirection).Vector;
            var thirdPoint = perpendicularTile.perp + thirdPointDirectionVector * (VisionRadius - 2);
            thirdPoint.DrawDebugLineFromRobot(_map, Color.blue);

            thirdPointDirectionVector = CardinalDirection.AngleToDirection((thirdPoint - perpendicularTile.point).GetAngleRelativeToX()).Vector;
            var thirdPointDirectionVectorPerpendicular = CardinalDirection.PerpendicularDirection(thirdPointDirectionVector).Vector;
            for (int i = 1; i < 4; i++)
            {
                var possibleUnseen = thirdPoint + (thirdPointDirectionVector * i) + thirdPointDirectionVectorPerpendicular;
                if (_map.IsWithinBounds(possibleUnseen) && _map.GetTileStatus(possibleUnseen) == SlamTileStatus.Unseen)
                {
                    perpendicularTile.perp.DrawDebugLineFromRobot(_map, Color.red);
                    _waypoint = new Waypoint(perpendicularTile.perp, Waypoint.WaypointType.Wall);
                    _controller.MoveTo(perpendicularTile.perp); //THIS FUCKED EVERYTHING
                    (CardinalDirection.AngleToDirection(0).Vector + _position).DrawDebugLineFromRobot(_map, Color.magenta);
                    return true;
                };
            }
            return false;
        }
        private List<Line2D> GetWalls(IEnumerable<Vector2Int> tiles)
        {
            List<Line2D> lines = new();
            foreach (var tile in tiles)
            {
                foreach (var otherTile in tiles)
                {
                    if (tile != otherTile)
                    {
                        var line = new Line2D(tile, otherTile);
                        if (!lines.Contains(line))
                        {
                            lines.Add(line);
                        }
                    }
                }
            }

            var superLines = new List<Line2D>();
            foreach (var line in lines)
            {
                var isSuperLine = true;
                var otherLines = lines.Where(otherLine => line != otherLine); 
                foreach (var otherLine in otherLines)
                {
                    if (otherLine.Contains(line))
                    {
                        isSuperLine = false;
                        break;
                    }
                }
                if (isSuperLine)
                { 
                    superLines.Add(line);
                }
            }

            // Sort the list for the longest lines first, since it will make the distinct (same a and b in ax+b) remainder the longest line.
            lines = superLines.OrderByDescending(line => Vector2.Distance(line.Start, line.End)).Distinct().ToList();
            List<Line2D> results = new();
            foreach (var line in lines)
            {
                var isUnbroken = true;
                var points = line.Rasterize();
                if (points.Any(tile => _map.GetTileStatus(tile) != SlamTileStatus.Solid))
                {
                    isUnbroken = false;
                }
                if (isUnbroken)
                {
                    results.Add(line);
                }
            }
            return results.Distinct().ToList();
        }


        private bool MoveToCornerCoverage()
        {
            if (IsAheadExploreable(1) && IsAheadExploreable(2)) return false;
            var tiles = _edgeDetector.GetTilesAroundRobot(VisionRadius + 2, new List<SlamTileStatus> { SlamTileStatus.Unseen, SlamTileStatus.Solid }, 360 - 30);
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
                _waypoint = new Waypoint(location, Waypoint.WaypointType.Corner);
                return true;
            }
            return false;
        }

        private bool MoveToNearestEdge()
        {
            var tiles = _edgeDetector.GetBoxAroundRobot()
                              .Where(tile => (_map.GetTileStatus(tile) != SlamTileStatus.Unseen)
                                             && IsPerpendicularUnseen(tile))
                              .Select(tile => (status: _map.GetTileStatus(tile), perpendicularTile: tile + CardinalDirection.PerpendicularDirection(tile - _position).Vector * (VisionRadius - 1)))
                              .Where(tile => _map.IsWithinBounds(tile.perpendicularTile) && IfSolidIsPathable(tile))
                              .Select(tile => tile.perpendicularTile);
            if (tiles.Any())
            {
                // Take candidateTile from local right sweeping counter clockwise
                var localRight = _controller.GetGlobalAngle() + 180 % 360;
                var closestTile = tiles.OrderBy(tile => ((tile - _position).GetAngleRelativeToX() - localRight + 360) % 360).First();
                var angle = Vector2.SignedAngle(Vector2.right, closestTile - _position);
                var vector = Geometry.VectorFromDegreesAndMagnitude(angle, VisionRadius + 1);
                closestTile = Vector2Int.FloorToInt(vector + _position);
                foreach (var tile in tiles)
                {
                    tile.DrawDebugLineFromRobot(_map, Color.yellow);
                }
                closestTile.DrawDebugLineFromRobot(_map, Color.white);
                _controller.MoveTo(closestTile);
                _waypoint = new Waypoint(closestTile, Waypoint.WaypointType.Edge);
                return true;
            }
            return false;
        }

        private bool IsPerpendicularUnseen(Vector2Int tile)
        {
            var direction = CardinalDirection.PerpendicularDirection(tile - _position).Vector;
            var perp = direction + tile;
            if (!_map.IsWithinBounds(perp)) return false;
            return _map.GetTileStatus(perp) == SlamTileStatus.Unseen;
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
            var tile = _map.GetNearestTileFloodFill(_position, SlamTileStatus.Unseen);
            if (tile.HasValue)
            {
                tile = _map.GetNearestTileFloodFill(tile.Value, SlamTileStatus.Open);
                if (tile.HasValue)
                {
                    _controller.PathAndMoveTo(tile.Value);
                    _waypoint = new Waypoint(tile.Value, Waypoint.WaypointType.Greed, true);
                    tile.Value.DrawDebugLineFromRobot(_map, Color.cyan);
                }
            }
            else
            {
                _currentState = AlgorithmState.Auctioning;
            }
        }

        private bool IsDestinationReached()
        {
            return _waypoint.HasValue && _map.GetTileCenterRelativePosition(_waypoint.Value.Destination).Distance < 0.5f;
        }

        private bool IsAheadExploreable(int range)
        {
            var direction = CardinalDirection.AngleToDirection(_controller.GetGlobalAngle());
            var target = direction.Vector * (VisionRadius + range) + _position;
            if (_map.IsWithinBounds(target))
            {
                return _map.GetTileStatus(target) == SlamTileStatus.Unseen;
            }
            return false;
        }

        private bool IsAroundExplorable(int range)
        {
            return _edgeDetector.GetTilesAroundRobot(VisionRadius + range, new List<SlamTileStatus> { SlamTileStatus.Solid })
                                .Count(tile => _map.GetTileStatus(tile) == SlamTileStatus.Unseen) > 0;
        }

        private List<RelativeWall> GetWallsNearRobot()
        {
            return _visibleTiles.Where(kv => kv.Value == SlamTileStatus.Solid)
                                     .Select(kv => _map.FromSlamMapCoordinate(kv.Key))
                                     .Distinct()
                                     .Select(position =>
                                         new RelativeWall
                                         {
                                             Position = position,
                                             Distance = Vector2.Distance(position, _position),
                                             Angle = ((position - _position).GetAngleRelativeToX() - _controller.GetGlobalAngle() + 360) % 360
                                         }
                                     )
                                     .OrderBy(dist => dist.Distance)
                                     .ToList();
        }

        private DoorState DoorwayDetection(List<RelativeWall> visibleWallTiles)
        {
            if (!visibleWallTiles.Any()) return DoorState.None;
            var visibleTiles = _visibleTiles.Select(kv => _map.FromSlamMapCoordinate(kv.Key)).Distinct();
            var wallTilePositions = visibleWallTiles.Select(wall => wall.Position);
            var walls = GetWalls(wallTilePositions);

            if (walls.Count == 1)
            {
                var (start, end) = SortSingleWall(walls);
                var wallDirectionVector = CardinalDirection.DirectionFromDegrees((end - start).GetAngleRelativeToX()).Vector;
                var hasOpening = Enumerable.Range(0, VisionRadius * 2)
                          .Select(r => Vector2Int.FloorToInt(start + wallDirectionVector * r))
                          .Where(tile => visibleTiles.Contains(tile))
                          .Any(tile => _map.GetTileStatus(tile) == SlamTileStatus.Open);

                if (hasOpening)
                {
                    end.DrawDebugLineFromRobot(_map, Color.magenta);
                    var distanceToEnd = (end - _position) * new Vector2Int(Mathf.Abs(wallDirectionVector.x), Mathf.Abs(wallDirectionVector.y));
                    _waypoint = new Waypoint(_position + distanceToEnd + wallDirectionVector * _doorWidth, Waypoint.WaypointType.Door);
                    _controller.MoveTo(_waypoint.Value.Destination);
                    _waypoint.Value.Destination.DrawDebugLineFromRobot(_map, Color.green);
                    _lastWallTile = end;
                    return DoorState.Single;
                }
            }


            return DoorState.None;
        }

        private (Vector2Int start, Vector2Int end) SortSingleWall(List<Line2D> walls)
        {
            var wall = walls.First();
            var orderedPoints = new List<Vector2Int> { Vector2Int.FloorToInt(wall.Start), Vector2Int.FloorToInt(wall.End) }
                            .OrderBy(tile => ((tile - _position).GetAngleRelativeToX() - (_controller.GetGlobalAngle() + 180) % 360 + 360) % 360);
            return (orderedPoints.First(), orderedPoints.Last());

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
            if (_closestDoorway == null) return;
            int doorwayDistance = int.MaxValue;
            bool didNotPassDoorway = true;
            foreach (Doorway doorway in _doorways)
            {
                List<Vector2Int> path = _controller.GetSlamMap().GetPath(_position, doorway.Center);
                var distance = path.Count;
                if (distance < doorwayDistance)
                {
                    for (int i = 0; i < distance - 1; i++)
                    {
                        foreach (Doorway doorwayForPath in _doorways)
                        {
                            if (path[i] == doorwayForPath.Center) didNotPassDoorway = false;
                        }
                    }
                    if (didNotPassDoorway)
                    {
                        doorwayDistance = distance;
                        _closestDoorway = doorway;
                    }
                };
            }
            _controller.PathAndMoveTo(_closestDoorway.Center);
            if (_position == _closestDoorway.Center)
            {
                _currentState = AlgorithmState.Idle;
                _closestDoorway = null;
            }
        }

        private void MoveThroughNearestUnexploredDoorway()
        {
            //If guard clauses could be avoided, that would be great
            if (_closestDoorway == null) return;
            int doorwayDistance = int.MaxValue;
            foreach (Doorway doorway in _doorways)
            {
                List<Vector2Int> path = _controller.GetSlamMap().GetPath(_position, doorway.Center);
                var distance = path.Count;
                if (distance < doorwayDistance)
                {
                    doorwayDistance = distance;
                    _closestDoorway = doorway;
                }
            }
            _controller.PathAndMoveTo(_closestDoorway.Center);
            if (_position == _closestDoorway.Center)
            {
                _currentState = AlgorithmState.Idle;
                _closestDoorway = null;
            }
        }
    }
}
