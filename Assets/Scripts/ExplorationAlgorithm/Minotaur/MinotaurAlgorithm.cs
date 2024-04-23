using Maes.Map;
using Maes.Robot;
using System;
using System.Linq;
using System.Collections.Generic;
using UnityEngine;
using Maes.Utilities;
using static Maes.Map.SlamMap;
using Maes.Map.PathFinding;
using UnityEditor.Experimental.GraphView;

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
        private Vector2Int _position => _map.GetCurrentPosition();
        protected List<Doorway> _doorways = new();
        private List<MinotaurAlgorithm> _minotaurs;
        private bool _clockwise = false;
        private AlgorithmState _currentState = AlgorithmState.Idle;
        private bool _potentialDoor = false;
        private bool _taskBegun;
        private Doorway _closestDoorway = null;
        private Waypoint? _waypoint;
        private Vector2Int? _lastWallTile;
        private List<Line2D> _lastWalls = new();
        private int _logicTicks = 0;
        private int _ticksSinceHeartbeat;
        private List<Vector2Int> _previousIntersections = new();

        private enum AlgorithmState
        {
            Idle,
            FirstWall,
            ExploreRoom,
            Auctioning,
            MovingToDoorway,
            MovingToNearestUnexplored,
            Done
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
                Door,
                NearestDoor,
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
            Doorway.DoorWidth = _doorWidth;
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
            Doorway._map = _map;
            _edgeDetector = new EdgeDetector(_controller.GetSlamMap(), VisionRadius);
        }

        public void UpdateLogic()
        {
            _logicTicks++;
            _ticksSinceHeartbeat++;
            if (_ticksSinceHeartbeat == 10)
            {
                _ticksSinceHeartbeat = 0;
                _controller.Broadcast(new HeartbeatMessage(_controller.GetSlamMap(), _doorways, _position));
            }
            var receivedMessages = _controller.ReceiveBroadcast().OfType<IMinotaurMessage>();
            if (receivedMessages.Count() > 1)
            {
                var combinedMessage = receivedMessages.Take(1).First();
                foreach (var message in receivedMessages)
                {
                    combinedMessage.Combine(message, this);
                }
            } else if (receivedMessages.Any()) receivedMessages.First().Combine(receivedMessages.First(), this);



            if (_controller.IsCurrentlyColliding())
            {
                if (_controller.GetStatus() != Robot.Task.RobotStatus.Idle)
                    _controller.StopCurrentTask();
                else
                    _controller.Move(1, true);
                _waypoint = null;
                _potentialDoor = false;
                return;
                //TODO: full resets
            }

            var wallPoints = GetWallsNearRobot();
            if (!_waypoint.HasValue)
            {
                DoorwayDetection(wallPoints);
            }
            if (_waypoint.HasValue)
            {
                var waypoint = _waypoint.Value;

                if (waypoint.Type != Waypoint.WaypointType.Door && waypoint.Type != Waypoint.WaypointType.NearestDoor)
                    DoorwayDetection(wallPoints);

                if (waypoint.UsePathing)
                {
                    _controller.PathAndMoveTo(waypoint.Destination);
                }
                else
                {
                    var solidTile = _edgeDetector.GetFurthestTileAroundRobot((waypoint.Destination - _position).GetAngleRelativeToX(), VisionRadius - 2, new List<SlamTileStatus> { SlamTileStatus.Solid }, true);
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
                    if (waypoint.Type == Waypoint.WaypointType.Door)
                        AttemptAddDoorway(GetWalls(wallPoints.Select(wallPoint => wallPoint.Position).Distinct()));
                    _waypoint = null;
                }
                else
                    return;
            }

            switch (_currentState)
            {
                case AlgorithmState.Idle:
                    _controller.StartMoving();
                    _currentState = AlgorithmState.FirstWall;
                    break;
                case AlgorithmState.FirstWall:
                    if (wallPoints.Any() && wallPoints.First().Distance / 2 < VisionRadius - 1)
                    {
                        _controller.StopCurrentTask();
                        _currentState = AlgorithmState.ExploreRoom;
                    }
                    break;
                case AlgorithmState.ExploreRoom:
                    if (_controller.GetStatus() == Robot.Task.RobotStatus.Idle)
                    {
                        if (!IsAroundExplorable(2))
                        {
                            if (MoveToNearestUnseenWithinRoom()) break;
                            else if (MovetoNearestDoorway()) break;
                            else if (MoveToNearestUnseen()) break;
                            else _currentState = AlgorithmState.Done;
                        }
                        else if (MoveAlongWall()) break;
                        else if (MoveToCornerCoverage()) break;
                        else if (MoveToNearestEdge()) break;
                        else if (MoveToNearestUnseenWithinRoom()) break;
                        else if (MovetoNearestDoorway()) break;
                        else if (MoveToNearestUnseen()) break;
                        else _currentState = AlgorithmState.Done;
                    }
                    break;
                case AlgorithmState.Auctioning:
                    break;
                case AlgorithmState.MovingToDoorway:
                    break;
                case AlgorithmState.Done:
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
            var slamMap = _controller.GetSlamMap();
            var slamPosition = slamMap.GetCurrentPosition();
            var slamTiles = _edgeDetector.GetTilesAroundRobot(VisionRadius + 2, new List<SlamTileStatus> { SlamTileStatus.Solid, SlamTileStatus.Unseen }, slamPrecision: true)
                                     .Where(tile => slamMap.GetTileStatus(tile) == SlamTileStatus.Solid)
                                     .ToList();
            slamTiles.AddRange(DoorwayTilesInRange(VisionRadius * 2));
            if (slamTiles.Count() < 2) return false;
            var coarseTileAhead = _edgeDetector.GetFurthestTileAroundRobot(_controller.GetGlobalAngle(), VisionRadius, new List<SlamTileStatus> { SlamTileStatus.Solid, SlamTileStatus.Unseen }, snapToGrid: true);
            var localLeft = (_controller.GetGlobalAngle() + (_map.GetTileStatus(coarseTileAhead) == SlamTileStatus.Solid ? 90 : 0)) % 360;
            (CardinalDirection.AngleToDirection(localLeft).Vector + _position).DrawDebugLineFromRobot(_map, Color.blue);
            var slamWalls = GetWalls(slamTiles);
            var slamWallPoints = slamWalls.Select(wall => Vector2Int.FloorToInt(wall.Start))
                              .Union(slamWalls.Select(wall => Vector2Int.FloorToInt(wall.End)));

            var furthestPoint = slamWallPoints.OrderByDescending(point => Vector2.Distance(point, slamPosition)).First();
            slamWallPoints = slamWallPoints.OrderByDescending(point => (Vector2.SignedAngle((furthestPoint - slamPosition), (point - slamPosition)) + 360) % 360);

            slamWalls.ToList().ForEach(wall => Debug.DrawLine(slamMap.TileToWorld(wall.Start), slamMap.TileToWorld(wall.End), Color.white, 2));
            slamWallPoints.ToList().ForEach(point => point.DrawDebugLineFromRobot(slamMap, Color.yellow));

            var points = slamWallPoints.Select(point => (perp: point + Vector2Int.FloorToInt(Vector2.Perpendicular(point - slamPosition).normalized * (VisionRadius - 2) * 2), point))
                           .Where(tuple => slamMap.IsWithinBounds(tuple.perp)
                                           && slamMap.GetTileStatus(tuple.perp) != SlamTileStatus.Solid)
                           .ToList();

            if (!points.Any())
            {
                return false;
            }

            (Vector2Int perp, Vector2Int point)? openPoint = null;
            foreach (var point in points)
            {
                var tile = _edgeDetector.GetFurthestTileAroundRobot((point.perp - slamPosition).GetAngleRelativeToX(), (VisionRadius + 2) * 2, new List<SlamTileStatus> { SlamTileStatus.Solid }, slamPrecision: true);
                if ((point.perp - slamPosition).magnitude > (tile - slamPosition).magnitude)
                {
                    continue;
                }

                if (slamMap.IsWithinBounds(point.perp))
                {
                    if (slamMap.GetTileStatus(point.perp) == SlamTileStatus.Unseen)
                    {
                        if (WallUnseenNeighbor(point, slamWalls))
                            return true;
                    }
                    else if (slamMap.GetTileStatus(point.perp) == SlamTileStatus.Open)
                    {
                        openPoint = point;
                    }
                }
            }
            if (openPoint.HasValue)
            {
                return WallUnseenNeighbor(openPoint.Value, slamWalls);
            }

            return false;
        }

        private bool WallUnseenNeighbor((Vector2Int perp, Vector2Int point) point, List<Line2D> slamWalls)
        {
            var slamMap = _controller.GetSlamMap();
            var slamPosition = slamMap.GetCurrentPosition();
            var wall = slamWalls.First(wall => wall.Start == point.point || wall.End == point.point);
            var thirdPoint = wall.Rasterize().OrderBy(wallPoint => Vector2.Distance(wallPoint, slamPosition)).First();
            var towardRobotVector = CardinalDirection.VectorToDirection(slamPosition - thirdPoint).Vector;
            var wallDirection = CardinalDirection.VectorToDirection(point.point - thirdPoint).Vector;

            for (var i = 0; i < VisionRadius; i++)
            {
                var isWallTileUnseen = slamMap.GetTileStatus(point.point + (wallDirection * i) + towardRobotVector) == SlamTileStatus.Unseen;
                if (isWallTileUnseen)
                {
                    var destination = _map.FromSlamMapCoordinate(point.perp);
                    _waypoint = new Waypoint(destination, Waypoint.WaypointType.Wall, false);
                    destination.DrawDebugLineFromRobot(_map, Color.red);
                    _controller.MoveTo(destination);
                    return true;
                }
            }
            return false;
        }

        /// <summary>
        /// Gets the doorway tiles that are within range of the robot to a radius in coarse tiles<para/>
        /// Utilizing the equation of <see cref="Geometry.PointsWithinCircle(IEnumerable{Vector2Int}, Vector2, float)"/>
        /// </summary>
        /// <param name="radius">The radius in coarse tiles from the robot. R in the inequality</param>
        /// <returns>Doorway tiles around the robot within range</returns>
        private IEnumerable<Vector2Int> DoorwayTilesInRange(int radius)
        {
            var slamPosition = _controller.GetSlamMap().GetCurrentPosition();
            return Geometry.PointsWithinCircle(_doorways.SelectMany(doorway => doorway.Tiles), slamPosition, radius);
        }

        private List<Line2D> GetWalls(IEnumerable<Vector2Int> tiles)
        {
            if (tiles.Count() < 2) return new();
            var forwardAngle = _controller.GetGlobalAngle();
            var map = _controller.GetSlamMap();
            var slamPosition = map.GetCurrentPosition();

            var correctedTiles = new List<(Vector2Int corrected, Vector2Int original)>();
            foreach (var tile in tiles)
            {
                var correctedTile = tile;
                if (map.IsWithinBounds(tile + CardinalDirection.East.Vector) && map.GetTileStatus(tile + CardinalDirection.East.Vector) == SlamTileStatus.Open)
                {
                    if (map.IsWithinBounds(tile + CardinalDirection.West.Vector)
                        && map.GetTileStatus(tile + CardinalDirection.West.Vector) == SlamTileStatus.Solid
                        && map.IsWithinBounds(tile + CardinalDirection.North.Vector)
                        && map.GetTileStatus(tile + CardinalDirection.North.Vector) == SlamTileStatus.Open)
                    {
                        correctedTile = tile + CardinalDirection.NorthEast.Vector;
                    }
                    else
                    {
                        correctedTile = tile + CardinalDirection.East.Vector;
                    }
                }
                else if (map.IsWithinBounds(tile + CardinalDirection.North.Vector) && map.GetTileStatus(tile + CardinalDirection.North.Vector) == SlamTileStatus.Open)
                    correctedTile = tile + CardinalDirection.North.Vector;
                correctedTiles.Add((correctedTile, tile));
            }

            var leftAngleVector = CardinalDirection.AngleToDirection(forwardAngle + 90).Vector;
            var furthestPoint = correctedTiles.OrderByDescending(tile => Vector2.Distance(slamPosition, tile.corrected)).First().corrected;

            var sortedTiles = correctedTiles.Distinct()
                                            .OrderBy(tile => (Vector2.SignedAngle(furthestPoint - slamPosition, tile.corrected - slamPosition) + 360) % 360) // Should be 180 aka backwards
                                            .ThenByDescending(tile => Vector2.Distance(leftAngleVector * VisionRadius + slamPosition, tile.corrected))
                                            .ToArray();
            var test = sortedTiles.Select(tile => (Vector2.SignedAngle(leftAngleVector, tile.corrected - slamPosition) + 360) % 360);


            var startPoint = sortedTiles.First(); // Top right is borked
            var previousDirection = (startPoint.corrected - sortedTiles[1].corrected).GetAngleRelativeToX();

            var result = new List<Line2D>();
            for (int i = 0; i < sortedTiles.Count() - 1; i++)
            {
                var direction = (sortedTiles[i].corrected - sortedTiles[i + 1].corrected).GetAngleRelativeToX();
                if (previousDirection != direction || map.GetTileStatus(sortedTiles[i].original + CardinalDirection.AngleToDirection(direction).OppositeDirection().Vector) != SlamTileStatus.Solid)
                {
                    var originalLine = new Line2D(startPoint.original, sortedTiles[i].original);
                    if (originalLine.Rasterize().All(tile => map.IsWithinBounds(Vector2Int.FloorToInt(tile))
                                                             && map.GetTileStatus(Vector2Int.FloorToInt(tile)) == SlamTileStatus.Solid))
                    {
                        result.Add(new Line2D(startPoint.corrected, sortedTiles[i].corrected));
                    }
                    if (i != 0 && map.GetTileStatus(sortedTiles[i + 1].original + CardinalDirection.AngleToDirection(direction).Vector) == SlamTileStatus.Open)
                    {
                        startPoint = sortedTiles[i + 1];
                    }
                    else
                    {
                        startPoint = sortedTiles[i];
                    }
                    previousDirection = direction;
                }
            }
            var lastLine = new Line2D(startPoint.original, sortedTiles.Last().original);
            if (lastLine.Rasterize().All(tile => map.IsWithinBounds(Vector2Int.FloorToInt(tile)) && map.GetTileStatus(Vector2Int.FloorToInt(tile)) == SlamTileStatus.Solid))
            {
                result.Add(new(startPoint.corrected, sortedTiles.Last().corrected));
            }
            return result;
        }

        private List<(Vector2Int intersection, List<Line2D> walls)> GetIntersectionPoints(IPathFindingMap map, IEnumerable<Line2D> walls)
        {
            List<(Vector2Int intersection, List<Line2D> walls)> intersectionPoints = new();
            foreach (var line in walls)
            {
                var otherlines = walls.Where(tempLine => tempLine != line);
                foreach (var otherline in otherlines)
                {
                    var intersectionPoint = line.GetIntersection(otherline, true);
                    if (intersectionPoint.HasValue)
                    {
                        intersectionPoints.Add((Vector2Int.FloorToInt(intersectionPoint.Value), new List<Line2D> { line, otherline }));
                    }
                }
            }
            return intersectionPoints.Where(point => map.IsWithinBounds(point.intersection))
                                     .GroupBy(point => point.intersection)
                                     .Select(groupPoints => groupPoints.First())
                                     .ToList();
        }

        private bool MoveToCornerCoverage()
        {
            if (IsAheadExploreable(1) && IsAheadExploreable(2)) return false;
            var tiles = _edgeDetector.GetTilesAroundRobot(VisionRadius + 2, new List<SlamTileStatus> { SlamTileStatus.Unseen, SlamTileStatus.Solid }, startAngle: 360 - 30);
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

        private bool MoveToNearestUnseenWithinRoom()
        {
            var doorTiles = _doorways.SelectMany(doorway => _map.FromSlamMapCoordinates(doorway.Tiles.ToList())).ToHashSet();
            var startCoordinate = _position;
            if (_map.GetTileStatus(startCoordinate) == SlamTileStatus.Solid)
            {
                var NearestOpenTile = _map.GetNearestTileFloodFill(startCoordinate, SlamTileStatus.Open, doorTiles);
                if (NearestOpenTile.HasValue)
                {
                    startCoordinate = NearestOpenTile.Value;
                }
            }
            var tile = _map.GetNearestTileFloodFill(startCoordinate, SlamTileStatus.Unseen, doorTiles);
            if (tile.HasValue)
            {
                tile = _map.GetNearestTileFloodFill(tile.Value, SlamTileStatus.Open);
                if (tile.HasValue)
                {
                    _controller.PathAndMoveTo(tile.Value);
                    _waypoint = new Waypoint(tile.Value, Waypoint.WaypointType.Greed, true);
                    return true;
                }
            }
            return false;
        }

        private bool MovetoNearestDoorway()
        {
            var nearestDoorway = GetNearestUnexploredDoorway();
            if (nearestDoorway != null)
            {
                _waypoint = new Waypoint(_map.FromSlamMapCoordinate(nearestDoorway.Center + nearestDoorway.ApproachedDirection.Vector * 4), Waypoint.WaypointType.NearestDoor, true);
                _controller.PathAndMoveTo(_waypoint.Value.Destination);
                nearestDoorway.Explored = true;
                return true;
            }
            return false;
        }

        private bool MoveToNearestUnseen()
        {
            var startCoordinate = _position;
            if (_map.GetTileStatus(startCoordinate) == SlamTileStatus.Solid)
            {
                var NearestOpenTile = _map.GetNearestTileFloodFill(startCoordinate, SlamTileStatus.Open);
                if (NearestOpenTile.HasValue)
                {
                    startCoordinate = NearestOpenTile.Value;
                }
            }
            var tile = _map.GetNearestTileFloodFill(startCoordinate, SlamTileStatus.Unseen);
            if (tile.HasValue)
            {
                _controller.PathAndMoveTo(tile.Value);
                _waypoint = new Waypoint(tile.Value, Waypoint.WaypointType.Greed, true);
                return true;
            }
            return false;
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
            return _edgeDetector.GetTilesAroundRobot(VisionRadius + range, new List<SlamTileStatus> { SlamTileStatus.Solid, SlamTileStatus.Unseen })
                                .Count(tile => _map.GetTileStatus(tile) == SlamTileStatus.Unseen) > 0;
        }

        private List<RelativeWall> GetWallsNearRobot()
        {
            var slamMap = _controller.GetSlamMap();
            return _edgeDetector.GetTilesAroundRobot(VisionRadius + 2, new List<SlamTileStatus> { SlamTileStatus.Solid, SlamTileStatus.Unseen }, true).Where(tile => slamMap.GetTileStatus(tile) == SlamTileStatus.Solid)
                                     .Distinct()
                                     .Select(tile =>
                                         new RelativeWall
                                         {
                                             Position = tile,
                                             Distance = Vector2.Distance(tile, _controller.GetSlamMap().GetCurrentPosition()),
                                             Angle = ((tile - _position).GetAngleRelativeToX() - _controller.GetGlobalAngle() + 360) % 360
                                         }
                                     )
                                     .OrderBy(dist => dist.Distance)
                                     .ToList();
        }

        private void DoorwayDetection(List<RelativeWall> visibleWallTiles)
        {
            if (!visibleWallTiles.Any()) return;
            var slamMap = _controller.GetSlamMap();
            var wallTilePositions = visibleWallTiles.Select(wall => wall.Position).Distinct();
            Vector2Int slamPosition = slamMap.GetCurrentPosition();
            var walls = GetWalls(wallTilePositions).Where(wall => wall.Start != wall.End);
            walls.ToList().ForEach(wall => Debug.DrawLine(slamMap.TileToWorld(wall.Start), slamMap.TileToWorld(wall.End), Color.white, 0.1f));

            if (walls.Count() == 1)
            {
                var wall = walls.First();
                var (start, end) = (Vector2Int.FloorToInt(wall.Start), Vector2Int.FloorToInt(wall.End));
                var wallDirectionVector = CardinalDirection.DirectionFromDegrees((end - start).GetAngleRelativeToX()).Vector;
                var closestWallPoint = wallTilePositions.OrderBy(tile => Vector2.Distance(slamPosition, tile)).First();
                var presumedUnbrokenLine = Enumerable.Range(0, VisionRadius * 2)
                          .Select(r => Vector2Int.FloorToInt(closestWallPoint + wallDirectionVector * r)).ToArray();
                var lineBroken = Geometry.PointsWithinCircle(presumedUnbrokenLine, slamPosition, VisionRadius * 2)
                          .Where(tile => slamMap.IsWithinBounds(tile))
                          .FirstOrDefault(tile => slamMap.GetTileStatus(tile) == SlamTileStatus.Open);

                if (lineBroken != default)
                {
                    if (_previousIntersections.Contains(lineBroken))
                    {
                        return;
                    }
                    _previousIntersections.Add(lineBroken);
                    end.DrawDebugLineFromRobot(slamMap, Color.magenta);
                    var distanceToEnd = (_map.FromSlamMapCoordinate(end) - _position) * new Vector2Int(Mathf.Abs(wallDirectionVector.x), Mathf.Abs(wallDirectionVector.y));
                    _waypoint = new Waypoint(_position + distanceToEnd + wallDirectionVector * _doorWidth, Waypoint.WaypointType.Door, false);
                    _controller.StopCurrentTask();
                    _controller.MoveTo(_waypoint.Value.Destination);
                    _waypoint.Value.Destination.DrawDebugLineFromRobot(_map, Color.green);
                    return;
                }
            }
            else
            {
                var intersectionPoints = GetIntersectionPoints(slamMap, walls).Distinct()
                                                                              .Where(point => !point.walls.All(wall => wall.Rasterize().Select(tile => Vector2Int.FloorToInt(tile)).Contains(point.intersection)))
                                                                              .Where(point => Geometry.IsPointWithinCirle(point.intersection, slamPosition, VisionRadius * 2));
                foreach (var intersection in intersectionPoints)
                {
                    if (_previousIntersections.Contains(intersection.intersection))
                    {
                        return;
                    }
                    _previousIntersections.Add(intersection.intersection);

                    if (slamMap.GetTileStatus(intersection.intersection) != SlamTileStatus.Solid)
                    {
                        Vector2Int destinationVector = intersection.intersection - slamPosition;
                        Vector2 visionVector = Geometry.VectorFromDegreesAndMagnitude(destinationVector.GetAngleRelativeToX(), VisionRadius);
                        var destination = _map.FromSlamMapCoordinate(Vector2Int.FloorToInt(destinationVector - visionVector) + slamPosition);
                        _waypoint = new(destination, Waypoint.WaypointType.Door, false);
                        _controller.StopCurrentTask();
                        _controller.MoveTo(_waypoint.Value.Destination);
                        _waypoint.Value.Destination.DrawDebugLineFromRobot(_map, Color.green);
                        return;
                    }
                }
            }
        }

        private void AttemptAddDoorway(List<Line2D> walls)
        {
            var slamMap = _controller.GetSlamMap();
            Vector2Int slamPosition = slamMap.GetCurrentPosition();
            var intersectionPoints = GetIntersectionPoints(slamMap, walls).Distinct();
            intersectionPoints = intersectionPoints.Where(point => !point.walls.All(wall => wall.Rasterize().Select(tile => Vector2Int.FloorToInt(tile)).Contains(point.intersection)));
            if (intersectionPoints.Any())
            {
                var intersectionPoint = intersectionPoints.First();
                var closest = GetClosestPoints(intersectionPoint.walls, intersectionPoint.intersection);
                var (start, end) = (closest.First(), closest.Last());
                var center = (start + end) / 2;
                if (slamMap.GetTileStatus(intersectionPoint.intersection) != SlamTileStatus.Unseen
                    && (Mathf.Approximately(Vector2.Distance(start, end), _doorWidth * 2) || (Vector2.Distance(start, end) <= _doorWidth * 2 && Vector2.Distance(start, end) >= 3)))
                {
                    var newDoorway = new Doorway(start, end, CardinalDirection.VectorToDirection(center - slamPosition));
                    if (_doorways.All(doorway => !doorway.Equals(newDoorway)))
                    {
                        Debug.Log($"doorway {start}-{end} at {_logicTicks}");
                        _doorways.Add(newDoorway);
                    }
                }
            }
        }

        private IEnumerable<Vector2Int> GetClosestPoints(List<Line2D> walls, Vector2Int queryPoint)
        {
            return walls.Select(wall => Vector2Int.FloorToInt(wall.Rasterize().OrderBy(tile => Vector2.Distance(tile, queryPoint)).First()));
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


        private Doorway? GetNearestUnexploredDoorway()
        {
            float doorwayDistance = float.MaxValue;
            Doorway nearestDoorway = null;
            var unexploredDoorways = _doorways.Where(doorway => !doorway.Explored);
            foreach (Doorway doorway in unexploredDoorways)
            {
                var distance = PathDistanceToPoint(_map.FromSlamMapCoordinate(doorway.Center));
                if (distance < doorwayDistance)
                {
                    doorwayDistance = distance;
                    nearestDoorway = doorway;
                };
            }
            return nearestDoorway;
        }

        /// <summary>
        /// Gets the real distance that the robot has to go to on the path to the point
        /// </summary>
        /// <param name="point">The point that gets pathed to</param>
        /// <returns>The distance of the path</returns>
        private float PathDistanceToPoint(Vector2Int point)
        {
            List<Vector2Int> path = _controller.GetSlamMap().GetPath(_position, point);
            if (path == null) return Mathf.Infinity;
            List<float> distances = new();
            for (var i = 0; i < path.Count - 2; i++)
            {
                var pathStep = path[i];
                if (pathStep == path.First())
                {
                    distances.Add(Vector2.Distance(path.First(), _position));
                }
                distances.Add(Vector2.Distance(pathStep, path[i + 1]));
            }
            return distances.Sum();
        }
    }
}
