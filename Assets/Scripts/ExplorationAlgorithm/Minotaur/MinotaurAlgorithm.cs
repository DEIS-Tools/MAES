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
using System.Runtime.InteropServices;

namespace Maes.ExplorationAlgorithm.Minotaur
{
    public partial class MinotaurAlgorithm : IExplorationAlgorithm
    {
        public int VisionRadius => (int)_robotConstraints.SlamRayTraceRange;

        private IRobotController _controller;
        private RobotConstraints _robotConstraints;
        private CoarseGrainedMap _map;
        private EdgeDetector _edgeDetector;
        private Dictionary<Vector2Int, SlamTileStatus> _visibleTiles => _controller.GetSlamMap().GetCurrentlyVisibleTiles();
        private int _seed;
        private Vector2Int _position => _map.GetCurrentPositionCoarseTile();
        private List<Doorway> _doorways;
        private List<MinotaurAlgorithm> _minotaurs;
        private CardinalDirection.RelativeDirection _followDirection = CardinalDirection.RelativeDirection.Right;
        private AlgorithmState _currentState = AlgorithmState.Idle;
        private bool _taskBegun;
        private Doorway _closestDoorway = null;
        private Waypoint? _waypoint;
        private RelativeWall _lastWall;
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

        private struct Waypoint
        {
            public Vector2Int Destination;
            public bool UsePathing;
            public Waypoint(Vector2Int destination, bool pathing = false)
            {
                Destination = destination;
                UsePathing = pathing;
            }
        }

        private struct RelativeWall
        {
            public Vector2Int Position;
            public float Distance;
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
            _logicTicks++;
            if (_controller.HasCollidedSinceLastLogicTick())
            {
                _controller.Move(1, true);
                _waypoint = null;
                return;
                //TODO: full resets
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
                ResetDestinationIfReached();
                return;
            }


            var wallPoints = GetWallsNearRobot();

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

            // var (minx, maxx, miny, maxy) = Geometry.GetBoundingBox(perpendicularTile.point, perpendicularTile.perp, thirdPoint);
            // for (int y = miny; y < maxy; y++)
            // {
            //     for (int x = minx; x < maxx; x++)
            //     {
            //         if ((perpendicularTile.point.x - perpendicularTile.perp.x) * (y - perpendicularTile.point.y) - (perpendicularTile.point.y - perpendicularTile.perp.y) * (x - perpendicularTile.point.x) >= 0 &&
            //         (perpendicularTile.perp.x - thirdPoint.x) * (y - perpendicularTile.perp.y) - (perpendicularTile.perp.y - thirdPoint.y) * (x - perpendicularTile.perp.x) >= 0 &&
            //         (thirdPoint.x - perpendicularTile.point.x) * (y - thirdPoint.y) - (thirdPoint.y - perpendicularTile.point.y) * (x - thirdPoint.x) >= 0)
            //         {
            //             var triangleTile = new Vector2Int(x, y);
            //             //Debug.Log($"X = {x}, Y = {y}, TileStatus = {_map.GetTileStatus(new Vector2Int(x, y))}");
            //             if (_map.IsWithinBounds(triangleTile) && _map.GetTileStatus(triangleTile) == SlamTileStatus.Unseen)
            //             {
            //                 new Vector2Int(x,y).DrawDebugLineFromRobot(_map, Color.magenta);

            //                 perpendicularTile.perp.DrawDebugLineFromRobot(_map, Color.red);
            //                 _waypoint = new Waypoint(perpendicularTile.perp);
            //                 _controller.MoveTo(perpendicularTile.perp);
            //                 (CardinalDirection.AngleToDirection(0).Vector + _position).DrawDebugLineFromRobot(_map, Color.magenta);
            //                 return true;
            //             }
            //         }
            //     }
            // }

            thirdPointDirectionVector = CardinalDirection.AngleToDirection((thirdPoint - perpendicularTile.point).GetAngleRelativeToX()).Vector;
            var thirdPointDirectionVectorPerpendicular = CardinalDirection.PerpendicularDirection(thirdPointDirectionVector).Vector;
            for (int i = 1; i < 4; i++)
            {
                var possibleUnseen = thirdPoint + (thirdPointDirectionVector * i) + thirdPointDirectionVectorPerpendicular;
                //if (_map.IsWithinBounds(possibleUnseen) && _map.GetTileStatus(possibleUnseen) == SlamTileStatus.Solid) 
                //{break;} //This line should make it a lot faster
                if (_map.IsWithinBounds(possibleUnseen) && _map.GetTileStatus(possibleUnseen) == SlamTileStatus.Unseen)
                {
                    perpendicularTile.perp.DrawDebugLineFromRobot(_map, Color.red);
                    if (_map.GetTileStatus(perpendicularTile.perp) == SlamTileStatus.Open)
                    {
                        //var freeTileBeforeWall = _edgeDetector.GetFurthestTileAroundRobot((perpendicularTile.perp - _position).GetAngleRelativeToX(), (int)Vector2Int.Distance(perpendicularTile.perp, _position), new List<SlamTileStatus>{SlamTileStatus.Solid}, false);
                        _waypoint = new Waypoint(perpendicularTile.perp, true);
                        _controller.PathAndMoveTo(perpendicularTile.perp);
                    }
                    else
                    {
                        _waypoint = new Waypoint(perpendicularTile.perp, false);
                        _controller.MoveTo(perpendicularTile.perp);
                    }

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
                        lines.Add(new Line2D(tile, otherTile));
                    }
                }
            }
            // Sort the list for the longest lines first, since it will make the distinct (same a and b in ax+b) remainder the longest line.
            lines = lines.OrderByDescending(line => Vector2.Distance(line.Start, line.End)).Distinct().ToList();
            List<Line2D> results = new();
            foreach (var line in lines)
            {
                var start = line.IsVertical ? line.Start.y : line.Start.x;
                var end = line.IsVertical ? line.End.y : line.End.x;
                if (start > end)
                {
                    (end, start) = (start, end);
                }

                var isUnbroken = true;
                for (float i = start; i < end; i += 0.5f)
                {
                    var tile = line.IsVertical ? new Vector2Int((int)line.SlopeIntercept(i), (int)i) : new Vector2Int((int)i, (int)line.SlopeIntercept(i));
                    if (_map.GetTileStatus(tile) != SlamTileStatus.Solid)
                    {
                        isUnbroken = false;
                        break;
                    }
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
                _waypoint = new Waypoint(location);
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
                _waypoint = new Waypoint(closestTile);
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
                    _waypoint = new Waypoint(tile.Value, true);
                    tile.Value.DrawDebugLineFromRobot(_map, Color.cyan);
                }
            }
            else
            {
                _currentState = AlgorithmState.Auctioning;
            }
        }

        private void ResetDestinationIfReached()
        {
            if (_map.GetTileCenterRelativePosition(_waypoint.Value.Destination).Distance < 0.5f)
            {
                _waypoint = null;
            }

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
                                     .Select(kv => new RelativeWall { Position = _map.FromSlamMapCoordinate(kv.Key), Distance = Vector2.Distance(_map.FromSlamMapCoordinate(kv.Key), _position) })
                                     .OrderBy(dist => dist.Distance)
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
                    List<Vector2Int> path = _controller.GetSlamMap().GetPath(_position, doorway.Position);
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
            if (_position == _closestDoorway.Position)
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
                    List<Vector2Int> path = _controller.GetSlamMap().GetPath(_position, doorway.Position);
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
            if (_position == _closestDoorway.Position)
            {
                _currentState = AlgorithmState.Idle;
                _closestDoorway = null;
            }
        }
    }
}
