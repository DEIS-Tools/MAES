// Copyright 2024 MAES
// 
// This file is part of MAES
// 
// MAES is free software: you can redistribute it and/or modify it under
// the terms of the GNU General Public License as published by the
// Free Software Foundation, either version 3 of the License, or (at your option)
// any later version.
// 
// MAES is distributed in the hope that it will be useful, but WITHOUT
// ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
// or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
// Public License for more details.
// 
// You should have received a copy of the GNU General Public License along
// with MAES. If not, see http://www.gnu.org/licenses/.
// 
// Contributors: Rasmus Borrisholt Schmidt, Andreas Sebastian Sørensen, Thor Beregaard
// 
// Original repository: https://github.com/Molitany/MAES

using Maes.Map;
using Maes.Robot;
using System;
using System.Linq;
using System.Collections.Generic;
using UnityEngine;
using Maes.Utilities;
using static Maes.Map.SlamMap;
using Maes.Map.PathFinding;

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
        private int _seed;
        private Vector2Int _position => _map.GetCurrentPosition();
        protected List<Doorway> _doorways = new();
        private AlgorithmState _currentState = AlgorithmState.Idle;
        private Waypoint? _waypoint;
        private int _logicTicks = 0;
        private int _ticksSinceHeartbeat;
        private HashSet<Vector2Int> _previousIntersections = new();
        private int _deadlockTimer = 0;
        private Vector2Int _previousPosition;
        private Waypoint _previousWaypoint;
        private int _auctionTicks;
        private Dictionary<int, (Vector2Int location, Vector2Int? destination)> _otherRobotPositions = new();
        private HashSet<Vector2Int> _otherRobotDestinations => _otherRobotPositions.Values.Where(position => position.destination.HasValue)
                                                                                          .Select(position => position.destination.Value)
                                                                                          .ToHashSet();
        private const int DEADLOCK_TIMEOUT = 5;
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
                Spread,
            }

            public Waypoint(Vector2Int destination, WaypointType type, bool pathing = false)
            {
                Destination = destination;
                Type = type;
                UsePathing = pathing;
            }

            public override bool Equals(object obj)
            {
                if (obj is Waypoint other)
                {
                    return Destination == other.Destination
                           && Type == other.Type
                           && UsePathing == other.UsePathing;
                }
                return false;
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
            _previousPosition = _position;
        }

        public void UpdateLogic()
        {
            _logicTicks++;
            _ticksSinceHeartbeat++;
            var receivedDoorwayFound = _controller.ReceiveBroadcast().OfType<DoorwayFoundMessage>();
            if (receivedDoorwayFound.Any())
            {
                foreach (DoorwayFoundMessage doorwayFound in receivedDoorwayFound)
                {
                    var bid = doorwayFound.Process(this);
                    if (bid != null)
                    {
                        _controller.Broadcast(bid);
                    }
                }
            }
            var receivedAuctionResult = _controller.ReceiveBroadcast().OfType<AuctionResultMessage>();
            if (receivedAuctionResult.Any())
            {
                foreach (var message in receivedAuctionResult)
                {
                    message.Process(this);
                }
            }
            if (_ticksSinceHeartbeat == 10)
            {
                var ownHeartbeat = new HeartbeatMessage(_controller.GetRobotID(), _controller.GetSlamMap(), _doorways, _position, _previousIntersections);
                _ticksSinceHeartbeat = 0;
                _controller.Broadcast(ownHeartbeat);
            }
            var receivedHeartbeat = new Queue<HeartbeatMessage>(_controller.ReceiveBroadcast().OfType<HeartbeatMessage>());
            if (receivedHeartbeat.Count > 1)
            {
                var combinedMessage = receivedHeartbeat.Dequeue();
                foreach (var message in receivedHeartbeat)
                {
                    combinedMessage = combinedMessage.Combine(message, this) as HeartbeatMessage;
                }
            }


            //if (_controller.IsCurrentlyColliding())
            //{
            //    if (_controller.GetStatus() != Robot.Task.RobotStatus.Idle)
            //        _controller.StopCurrentTask();
            //    else
            //    {
            //        var openTile = _map.GetNearestTileFloodFill(_position, SlamTileStatus.Open);
            //        if (openTile.HasValue)
            //            _controller.MoveTo(openTile.Value);
            //        else
            //            _controller.Move(1, true);
            //    }
            //    _waypoint = null;
            //    _potentialDoor = false;
            //    return;
            //    //TODO: full resets
            //}



            if (_deadlockTimer >= 5)
            {
                var waypoint = _waypoint;
                if (MoveToNearestUnseenWithinRoom()) ;
                else if (MovetoNearestDoorway()) ;
                else if (MoveToNearestUnseen(_otherRobotDestinations)) ;
                if (waypoint.HasValue && waypoint.Equals(_waypoint))
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
                }
                _deadlockTimer = 0;
            }
            //if (_otherRobotPositions.Any(robot => Vector2.Distance(robot.Value.location, _position) <= 2) && (!_waypoint.HasValue || _waypoint.Value.Type != Waypoint.WaypointType.NearestDoor))
            //{
            //    var nearRobot = new Vector2((float)_otherRobotPositions.Values.Average(robot => robot.location.x), (float)_otherRobotPositions.Values.Average(robot => robot.location.y));
            //    _waypoint = new Waypoint(_position - Vector2Int.FloorToInt((nearRobot - _position).normalized*2), Waypoint.WaypointType.Spread, true);
            //    _controller.MoveTo(_waypoint.Value.Destination);
            //}
            var wallPoints = GetWallsNearRobot();
            if (!_waypoint.HasValue)
            {
                DoorwayDetection(wallPoints);
            }
            if (_waypoint.HasValue)
            {
                var waypoint = _waypoint.Value;

                if (waypoint.Type == Waypoint.WaypointType.Greed)
                {
                    var tilesAtDestination = _edgeDetector.GetTilesAroundPoint(VisionRadius, new List<SlamTileStatus> { SlamTileStatus.Solid, SlamTileStatus.Unseen }, waypoint.Destination, slamPrecision: false);
                    if (!tilesAtDestination.Any(tile => _map.GetTileStatus(tile) == SlamTileStatus.Unseen))
                    {
                        if (MoveToNearestUnseenWithinRoom()) ;
                        else if (MovetoNearestDoorway()) ;
                        else if (MoveToNearestUnseen()) ;
                    }
                }

                if (waypoint.Type != Waypoint.WaypointType.Door && waypoint.Type != Waypoint.WaypointType.NearestDoor)
                    DoorwayDetection(wallPoints);

                _previousWaypoint = waypoint;
                if (waypoint.UsePathing)
                {
                    if (_map.GetPath(waypoint.Destination, false, false) == null)
                    {
                        MoveToNearestUnseen(_otherRobotDestinations.Union(new HashSet<Vector2Int> { waypoint.Destination }).ToHashSet());
                        waypoint = _waypoint.Value;
                    }
                    _controller.PathAndMoveTo(waypoint.Destination);
                }
                else
                {
                    var solidTile = _edgeDetector.GetFurthestTileAroundPoint((waypoint.Destination - _position).GetAngleRelativeToX(), VisionRadius - 2, new List<SlamTileStatus> { SlamTileStatus.Solid }, slamPrecision: false);
                    if (_map.GetTileStatus(solidTile) == SlamTileStatus.Solid)
                    {
                        _waypoint = null;
                        _controller.StopCurrentTask();

                        if (_logicTicks % DEADLOCK_TIMEOUT == 0)
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
                    _controller.MoveTo(waypoint.Destination);
                }
                if (IsDestinationReached())
                {
                    if (waypoint.Type == Waypoint.WaypointType.Door)
                        AttemptAddDoorway(GetWalls(wallPoints.Select(wallPoint => wallPoint.Position).Distinct()));
                    else if (waypoint.Type == Waypoint.WaypointType.NearestDoor)
                    {
                        var possibleFirstDoorway = _doorways.FirstOrDefault(doorway => _map.FromSlamMapCoordinate(doorway.Center + doorway.ExitDirection.Vector * 4) == waypoint.Destination);
                        if (possibleFirstDoorway != default) possibleFirstDoorway.Explored = true;
                    }
                    _waypoint = null;
                }
                else
                {
                    if (_logicTicks % DEADLOCK_TIMEOUT == 0)
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
                    if (wallPoints.Any())
                    {
                        _currentState = AlgorithmState.ExploreRoom;
                        break;
                    }
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
                            else if (MoveToNearestUnseen(_otherRobotDestinations)) break;
                            else _currentState = AlgorithmState.Done;
                        }
                        else if (MoveAlongWall()) break;
                        else if (MoveToCornerCoverage()) break;
                        else if (MoveToNearestEdge()) break;
                        else if (MoveToNearestUnseenWithinRoom()) break;
                        else if (MovetoNearestDoorway()) break;
                        else if (MoveToNearestUnseen(_otherRobotDestinations)) break;
                        else _currentState = AlgorithmState.Done;
                    }
                    break;
                case AlgorithmState.Auctioning: //Should probably wait a certain amount of ticks before doing this
                    _auctionTicks++;
                    if (_auctionTicks == 3)
                    {
                        _waypoint = null;
                        _controller.StopCurrentTask();
                        var receivedBids = new Queue<BiddingMessage>(_controller.ReceiveBroadcast().OfType<BiddingMessage>());
                        if (receivedBids.Any())
                        {
                            var combinedMessage = receivedBids.Dequeue();
                            foreach (var message in receivedBids)
                            {
                                combinedMessage = combinedMessage.Combine(message, this) as BiddingMessage;
                            }
                            var winnerMessage = combinedMessage.Process(this); //Causes the auction caller to move through doorway if enough robots in room
                            if (winnerMessage is AuctionResultMessage)
                            {
                                _controller.Broadcast(winnerMessage);
                            }
                        }
                        _currentState = AlgorithmState.ExploreRoom;
                    }
                    break;
                case AlgorithmState.MovingToDoorway:
                    break;
                case AlgorithmState.Done:
                    break;
                default:
                    break;
            }

            if (_logicTicks % DEADLOCK_TIMEOUT == 0)
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

        private bool MoveAlongWall()
        {
            // Wait until SlamMap has updated otherwise we see old data
            if (_logicTicks % _robotConstraints.SlamUpdateIntervalInTicks != 0)
            {
                return true;
            }
            var slamMap = _controller.GetSlamMap();
            var slamPosition = slamMap.GetCurrentPosition();
            var slamTiles = _edgeDetector.GetTilesAroundPoint(VisionRadius + 2, new List<SlamTileStatus> { SlamTileStatus.Solid, SlamTileStatus.Unseen }, slamPrecision: true)
                                     .Where(tile => slamMap.GetTileStatus(tile) == SlamTileStatus.Solid)
                                     .ToList();
            if (slamTiles.Count() < 2) return false;
            var coarseTileAhead = _edgeDetector.GetFurthestTileAroundPoint(_controller.GetGlobalAngle(), VisionRadius, new List<SlamTileStatus> { SlamTileStatus.Solid, SlamTileStatus.Unseen }, snapToGrid: true);
            var localLeft = (_controller.GetGlobalAngle() + (_map.GetTileStatus(coarseTileAhead) == SlamTileStatus.Solid ? 90 : 0)) % 360;
            (CardinalDirection.AngleToDirection(localLeft).Vector + _position).DrawDebugLineFromRobot(_map, Color.blue);
            var slamWalls = GetWalls(slamTiles);
            var slamWallPoints = slamWalls.Select(wall => Vector2Int.FloorToInt(wall.Start))
                              .Union(slamWalls.Select(wall => Vector2Int.FloorToInt(wall.End)));
            if (slamWallPoints.Count() < 2) return false;

            var furthestPoint = slamWallPoints.OrderByDescending(point => Vector2.Distance(point, slamPosition)).First();
            slamWallPoints = slamWallPoints.OrderByDescending(point => (Vector2.SignedAngle((furthestPoint - slamPosition), (point - slamPosition)) + 360) % 360);

            slamWalls.ToList().ForEach(wall => Debug.DrawLine(slamMap.TileToWorld(wall.Start), slamMap.TileToWorld(wall.End), Color.white, 2));
            slamWallPoints.ToList().ForEach(point => point.DrawDebugLineFromRobot(slamMap, Color.yellow));

            var doorways = DoorwaysInRange(VisionRadius * 2).Select(doorway => doorway.Opening);

            var points = slamWallPoints.Select(point => (perp: point + Vector2Int.FloorToInt((VisionRadius - 2) * 2 * Vector2.Perpendicular(point - slamPosition).normalized), point))
                           .Where(tuple => slamMap.IsWithinBounds(tuple.perp)
                                           && slamMap.GetTileStatus(tuple.perp) != SlamTileStatus.Solid
                                           && !IsPerpendicularThroughDoorway(tuple.perp, doorways))
                           .ToList();

            if (!points.Any())
            {
                return false;
            }

            (Vector2Int perp, Vector2Int point)? openPoint = null;
            foreach (var point in points)
            {
                var tile = _edgeDetector.GetFurthestTileAroundPoint((point.perp - slamPosition).GetAngleRelativeToX(), (VisionRadius + 2) * 2, new List<SlamTileStatus> { SlamTileStatus.Solid }, slamPrecision: true);
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

        private bool IsPerpendicularThroughDoorway(Vector2Int perp, IEnumerable<Line2D> doorways)
        {
            var slamMap = _controller.GetSlamMap();
            var slamPosition = slamMap.GetCurrentPosition();
            var path = new Line2D(slamPosition, perp);
            var doorwayPerpIntersections = doorways.Select(doorway => new List<Line2D> { doorway, path })
                                                 .SelectMany(intersect => GetIntersectionPoints(slamMap, intersect));
            var result = doorwayPerpIntersections.Any(doorwayPerpIntersection => doorwayPerpIntersection.walls.All(wall => wall.Rasterize().Select(tile => Vector2Int.FloorToInt(tile)).Contains(doorwayPerpIntersection.intersection)));
            return result;
        }

        private bool WallUnseenNeighbor((Vector2Int perp, Vector2Int point) point, List<Line2D> slamWalls)
        {
            var slamMap = _controller.GetSlamMap();
            var slamPosition = slamMap.GetCurrentPosition();
            var wall = slamWalls.FirstOrDefault(wall => wall.Start != wall.End && (wall.Start == point.point || wall.End == point.point));
            if (wall == default) 
                return false;
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
        private IEnumerable<Doorway> DoorwaysInRange(int radius)
        {
            var slamPosition = _controller.GetSlamMap().GetCurrentPosition();
            return _doorways.Where(doorway => doorway.Tiles.Any(tile => Geometry.IsPointWithinCirle(tile, slamPosition, radius)));
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
            var tiles = _edgeDetector.GetTilesAroundPoint(VisionRadius + 2, new List<SlamTileStatus> { SlamTileStatus.Unseen, SlamTileStatus.Solid }, startAngle: 360 - 30);
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
            var doorways = DoorwaysInRange(VisionRadius * 2).Select(doorway => doorway.Opening);
            var tiles = _edgeDetector.GetBoxAroundRobot()
                              .Where(tile => (_map.GetTileStatus(tile) != SlamTileStatus.Unseen)
                                             && IsPerpendicularUnseen(tile))
                              .Select(tile => (status: _map.GetTileStatus(tile), perpendicularTile: tile + CardinalDirection.PerpendicularDirection(tile - _position).Vector * (VisionRadius - 1)))
                              .Where(tile => _map.IsWithinBounds(tile.perpendicularTile)
                                             && IfSolidIsPathable(tile)
                                             && !IsPerpendicularThroughDoorway(tile.perpendicularTile, doorways))
                              .Select(tile => tile.perpendicularTile);
            if (tiles.Any())
            {
                var localRight = _controller.GetGlobalAngle() + 180 % 360;
                var closestTiles = tiles.OrderBy(tile => ((tile - _position).GetAngleRelativeToX() - localRight + 360) % 360);
                foreach (var closestTile in closestTiles)
                {
                    // Take candidateTile from local right sweeping counter clockwise
                    var angle = Vector2.SignedAngle(Vector2.right, closestTile - _position);
                    var vector = Geometry.VectorFromDegreesAndMagnitude(angle, VisionRadius + 1);
                    var candidateTile = Vector2Int.FloorToInt(vector + _position);
                    closestTile.DrawDebugLineFromRobot(_map, Color.yellow);
                    if (candidateTile == _previousWaypoint.Destination)
                        continue;
                    candidateTile.DrawDebugLineFromRobot(_map, Color.white);
                    _controller.MoveTo(closestTile);
                    _waypoint = new Waypoint(closestTile, Waypoint.WaypointType.Edge);
                    return true;
                }
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
            var doorTiles = _map.FromSlamMapCoordinates(_doorways.SelectMany(doorway => doorway.Tiles).ToList()).ToHashSet();
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
                _waypoint = new Waypoint(_map.FromSlamMapCoordinate(nearestDoorway.Center + nearestDoorway.ExitDirection.Vector * 4), Waypoint.WaypointType.NearestDoor, true);
                _controller.PathAndMoveTo(_waypoint.Value.Destination);
                return true;
            }
            return false;
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
                    _waypoint = new Waypoint(tile.Value, Waypoint.WaypointType.Greed, true);
                    return true;
                }
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
            return _edgeDetector.GetTilesAroundPoint(VisionRadius + range, new List<SlamTileStatus> { SlamTileStatus.Solid, SlamTileStatus.Unseen })
                                .Count(tile => _map.GetTileStatus(tile) == SlamTileStatus.Unseen) > 0;
        }

        private List<RelativeWall> GetWallsNearRobot()
        {
            var slamMap = _controller.GetSlamMap();
            return _edgeDetector.GetTilesAroundPoint(VisionRadius + 2, new List<SlamTileStatus> { SlamTileStatus.Solid, SlamTileStatus.Unseen }, slamPrecision: true).Where(tile => slamMap.GetTileStatus(tile) == SlamTileStatus.Solid)
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
                    if (_previousIntersections.Contains(lineBroken) || _doorways.Any(doorway => doorway.Tiles.Contains(lineBroken)))
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
                    if (_previousIntersections.Contains(intersection.intersection) || _doorways.Any(doorway => doorway.Tiles.Contains(intersection.intersection)))
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
                    var opening = new Line2D(start, end);
                    var doorDirection = CardinalDirection.VectorToDirection(start - end);
                    var extended = new Line2D(start*doorDirection.Vector*VisionRadius, end * doorDirection.OppositeDirection().Vector * VisionRadius);
                    var closestToRobot = GetClosestPoints(new List<Line2D> { extended }, slamPosition);
                    var newDoorway = new Doorway(opening, center, CardinalDirection.VectorToDirection(closestToRobot.First() - slamPosition));
                    var otherDoorway = _doorways.FirstOrDefault(doorway => doorway.Equals(newDoorway));

                    if (otherDoorway == null)
                    {
                        //Debug.Log($"doorway {start}-{end} at {_logicTicks}");
                        _doorways.Add(newDoorway);

                        _currentState = AlgorithmState.Auctioning;
                        _auctionTicks = 0;
                        _controller.Broadcast(new DoorwayFoundMessage(newDoorway, _controller.GetRobotID()));
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
                var distance = PathDistanceToPoint(doorway.Center);
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
            List<Vector2Int> path = _map.GetPath(_map.FromSlamMapCoordinate(point), false, false);
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
