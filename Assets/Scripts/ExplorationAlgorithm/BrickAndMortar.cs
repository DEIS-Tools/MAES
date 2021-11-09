#nullable enable
using System;
using System.Collections.Generic;
using System.Linq;
using Dora.MapGeneration;
using Dora.Robot;
using Dora.Robot.Task;
using Dora.Utilities;
using JetBrains.Annotations;
using UnityEditor;
using UnityEngine;
using Random = System.Random;

namespace Dora.ExplorationAlgorithm {
    public class BrickAndMortar: IExplorationAlgorithm {

        private RobotConstraints _constraints;
        private IRobotController _controller;
        private int _randomSeed;
        private Random _tagIdGenerator;

        private int depositedTagsCount = 0;
        
        // A safety measure to avoid being out of range due to inaccurate positioning by the robot
        private const float TolerablePositioningError = 0.2f;
        
        // The distance from a tag, at which the robot must be able to read all 8 neighbouring tags
        private const float FullViewDistance = 0.2f;
        
        // The maximum diagonal distance between two tags
        private readonly float _maximumDiagonalDistance;
        private readonly float _minimumDiagonalDistance;
        // The maximum distance between along either axis (x or y)
        private readonly float _maximumAxialDistance;
        private readonly float _minimumAxialDistance;

        private readonly float _preferredAxialDistance;
        private readonly float _preferredDiagonalDistance;
        
        private BrickAndMortarTag? _currentTile;
        private NeighbourTile? _targetTile;
        private bool _allTilesVisited = false;
        private int _previousDirection = 0;

        private int _robotID = -1;
        
        private Stack<int> _controlledTiles = new Stack<int>();

        // The order of directions will prefer to go
        private readonly int[] _directionPriority = new[] { 0, 2, 4, 6};

        private AlgorithmState _currentState = AlgorithmState.Regular;
        private enum AlgorithmState {
            Regular,
            LoopControl,
            LoopClosing,
            LoopCleaning
        }
        
        public enum TileStatus {
            // Unexplored indicates that no robot has been here
            // Explored indicated that one or more robots have been here, but may need to traverse it again
            // Visited indicates that this tile is fully explored and need not be visited later (effectively a wall)
            // Solid indicate that the tile is physically in-traversable
            Unexplored = 0, Explored = 1, Visited = 2, Solid = 3
        }

        // Represents an immediate neighbour of a tag/tile 
        private class NeighbourTile {
            private BrickAndMortar _bm;
            
            // The tile may be solid, unexplored, explored or visisted
            public TileStatus Status;

            // The center tag of which this tile is a neighbour
            public BrickAndMortarTag CenterTag;
            // The neighbour tag, if it exists
            public BrickAndMortarTag? Tag;
            
            // Direction of the neighbour from the center tag/tile
            public int DirectionFromCenter;

            public NeighbourTile(BrickAndMortar bm, TileStatus status, int directionFromCenter, 
                BrickAndMortarTag centerTag, BrickAndMortarTag? neighbour) {
                Status = status;
                CenterTag = centerTag;
                Tag = neighbour;
                _bm = bm;
                DirectionFromCenter = directionFromCenter;
            }

            // Returns the position of this tile relative to the robot
            public RelativePosition<NeighbourTile> GetRelativePosition() {
                var tags = _bm.GetNearbyTags();
                if (Tag != null) {
                    // Just get the relative position of the tag from the robot sensors
                    return tags.First(t => t.Item.ID == Tag.ID).Map(_ => this);
                } else {
                    // Calculate the relative position of this tile based on the relative position to the center
                    var centerRelativeToRobot = tags.First(t => t.Item.ID == CenterTag.ID);
                    var neighbourAngle = DirectionToAngle(DirectionFromCenter);
                    var robotGlobalAngle = _bm._controller.GetGlobalAngle();
                    
                    // The distance to this tile depends on whether it is diagonal from the center
                    var neighbourDistance = IsDirectionDiagonal(DirectionFromCenter)
                        ? _bm._preferredDiagonalDistance
                        : _bm._preferredAxialDistance;
                    var neighbourRelativeToCenter = new RelativePosition<BrickAndMortarTag?>(neighbourDistance, neighbourAngle, null);
                    return _bm.GetNeighbourPosRelativeToRobot(centerRelativeToRobot, neighbourRelativeToCenter, robotGlobalAngle)
                        .Map(_ => this);
                }
            }

            public bool IsTraversable() {
                return Status == TileStatus.Unexplored || Status == TileStatus.Explored;
            }
        }

        // TODO: Convert Direction to a class
        private const int CardinalDirectionsCount = 8;
        // Index representing 8 neighbouring tags/tiles
        private const int
            East = 0,
            SouthEast = 1,
            South = 2,
            SouthWest = 3,
            West = 4,
            NorthWest = 5,
            North = 6,
            NorthEast = 7;
        
        private static int OppositeDirection(int direction) => (direction + 4) % 8;
        private static int DirectionToAngle(int direction) => ((8 - direction) % 8) * 45;
        private static bool IsDirectionDiagonal(int direction) => direction % 2 != 0;
        
        public BrickAndMortar(RobotConstraints constraints, int randomSeed) {
            _constraints = constraints;
            _randomSeed = randomSeed;
            _tagIdGenerator = new Random(randomSeed);
            
            // The maximum diagonal distance between two points is 1.5 tiles
            _maximumDiagonalDistance = constraints.EnvironmentTagReadRange - TolerablePositioningError - 2f;
            // Axial distance formula derived from a² + b² = c²  (And since we operate in a grid we have a = b)
            // This gives   2a² = c²  then rearranging to:  a = sqrt(c²/2)
            _maximumAxialDistance = Mathf.Sqrt(Mathf.Pow(_maximumDiagonalDistance, 2f) / 2f);

            _preferredDiagonalDistance = _maximumDiagonalDistance * (2f/3f);
            _preferredAxialDistance = _maximumAxialDistance * (2f/3f);

            _minimumDiagonalDistance = _maximumDiagonalDistance * (1f / 3f);
            _minimumAxialDistance = _maximumAxialDistance * (1f / 3f);
        }

        // Main loop of the algorithm
        public void UpdateLogic() {
            if (_allTilesVisited || _controller.GetStatus() != RobotStatus.Idle)
                return;

            // If uninitialized, then init the algorithm by dropping tag at current position
            _currentTile ??= DepositNewTag();
            
            // If no target tile is currently chosen, then update the current tile status and find next tile to visit
            if (_targetTile == null) {
                var neighbours = GetNeighbours(_currentTile);

                // Update status of current tile
                UpdateTileStatus(_currentTile, neighbours, _previousDirection);

                // Find next target tile
                _targetTile ??= DetermineNextTarget(neighbours);
                if (_targetTile == null) {
                    // No tiles left to explore, exploration has been completed
                    _allTilesVisited = true;
                    return;
                }
            }

            var targetRelativePosition = _targetTile.GetRelativePosition();

            // If we are not already at this tile, start rotating/moving towards it
            if (targetRelativePosition.Distance > TolerablePositioningError) {
                MoveToTag(targetRelativePosition);
                return;
            }
            
            // If there is no tag at this tile, ie. it is unexplored, place a new tag and use that instead
            _targetTile.Tag ??= DepositNewTag();
            // Update neighbour information for this tag and its neighbours
            UpdateTagNeighbours(_targetTile, _currentTile);

            // Note which direction we took this step, to avoid going back the same way
            _previousDirection = _targetTile.DirectionFromCenter;
            _currentTile.SetLastExitDirection(_controller.GetRobotID(), _previousDirection);

            // Lastly, update the 'current tag' to be the one we are currently standing on and clear the target tile
            _currentTile = _targetTile.Tag;
            _targetTile = null;
        }

        private void UpdateTileStatus(BrickAndMortarTag tile, NeighbourTile[] neighbours, int enterDirection) {
            if (_currentState == AlgorithmState.Regular) {
                // Check if tile can be marked as visited
                if (!CenterBlocksPathBetweenNeighbours(neighbours))
                    tile.Status = TileStatus.Visited;
                else {
                    var previousExitDirection = tile.GetLastExitDirection(_controller.GetRobotID());
                    if (previousExitDirection != null && OppositeDirection(enterDirection) != previousExitDirection)
                        _currentState = AlgorithmState.LoopControl;
                }
                return;
            }

            if (_currentState == AlgorithmState.LoopControl) {
                var lastExitDirection = tile.GetLastExitDirection(_controller.GetRobotID());
                if (lastExitDirection == null || OppositeDirection(enterDirection) == lastExitDirection) {
                    _currentState = AlgorithmState.LoopCleaning;
                }
                else {
                    var controllingRobot = tile.LoopController;
                    if (controllingRobot == _robotID) {
                        // Start loop closing
                        _currentState = AlgorithmState.LoopClosing;
                        // Reverse the controlled tiles stack when first entering the loop closing phase,
                        // as we want to close the tiles in the direction we initially controlled them
                        _controlledTiles.Reverse();
                        _controlledTiles.Pop();
                        return;
                        Debug.Log("STARTED LOOP CLOSING");
                    } else if (controllingRobot < _robotID) {
                        // Take control of the loop
                        tile.LoopController = _robotID;
                        _controlledTiles.Push(tile.ID);
                    } else if (controllingRobot > _robotID) {
                        // Start loop cleaning if this robot was in loop control phase and another robot with higher id
                        // is trying to control the same tiles
                        if (_controlledTiles.Count > 0) {
                            _currentState = AlgorithmState.LoopCleaning;
                            Debug.Log("STARTED LOOP CLEANING 1");
                        }
                    }
                }
            }

            if (_currentState == AlgorithmState.LoopClosing) {
                // Loop closing phase
                // Detect if we are at an 'intersection' (a tile with traversable neighbour tile(s) that
                // does not belong to the loop). If not, mark the cell as visited
                if (!HasUncontrolledNeighbour(neighbours)) {
                    tile.Status = TileStatus.Visited;
                } else {
                    // Stop loop controlling and start loop cleaning
                    _currentState = AlgorithmState.LoopCleaning;
                    tile.Clean(_robotID);
                    Debug.Log("STARTED LOOP CLEANING 2");
                }
            }
            
            if (_currentState == AlgorithmState.LoopCleaning) {
                tile.Clean(_robotID);
            }
        }

        // Returns true if any of the given neighbours are not 
        private bool HasUncontrolledNeighbour(NeighbourTile[] neighbours) {
            return neighbours.Any(n => n.IsTraversable() && n.Tag != null && n.Tag.LoopController != _robotID);
        }

        private NeighbourTile? DetermineNextTarget(NeighbourTile[] neighbours) {
            if (_currentState == AlgorithmState.LoopCleaning || _currentState == AlgorithmState.LoopClosing) {
                if (_controlledTiles.Count == 0) {
                    // Loop cleaning completed
                    _currentState = AlgorithmState.Regular;
                } else {
                    var tile = _controlledTiles.Pop();
                    return neighbours.First(n => n.Tag != null && n.Tag!.ID == tile);
                }
            }
            
            int bestTileIndex = 0;
            var bestScore = Int32.MinValue;
            var bestDirPriority = 5;

            for (int i = 0; i < CardinalDirectionsCount; i+=2) {
                // Ignore visited and solid tiles
                if (!neighbours[i].IsTraversable())
                    continue;
                
                var score = 0;
                
                // Prioritize not going the same direction that you came from, unless necessary
                if (OppositeDirection(i) == _previousDirection)
                    score -= 100;
                
                // Prefer unexplored tiles over explored ones
                if (neighbours[i].Status == TileStatus.Unexplored) score += 10;

                // Check if the two neighbours of this tile are solid/visited, if so increase the score
                if (!neighbours[(i + 7) % CardinalDirectionsCount].IsTraversable()) score++;
                if (!neighbours[(i + 1) % CardinalDirectionsCount].IsTraversable()) score++;

                int directionPriority = Array.IndexOf(_directionPriority, i);
                if (score > bestScore || (score == bestScore && directionPriority < bestDirPriority)) {
                    bestScore = score;
                    bestTileIndex = i;
                    bestDirPriority = directionPriority;
                }
            }
            
            // If no neighbour is traversable, the algorithm must have terminated
            if (bestScore == -1) 
                return null;
            
            return neighbours[bestTileIndex];
        }

        // Returns true if the center tile must be traversed to travel between two of the given neighbours
        private bool CenterBlocksPathBetweenNeighbours(NeighbourTile[] neighbours) {
            // First find all tiles reachable via the center
            HashSet<int> tilesReachableViaCenter = new HashSet<int>();
            for (int direction = 0; direction < CardinalDirectionsCount; direction+=2) {
                if (neighbours[direction].IsTraversable()) {
                    tilesReachableViaCenter.Add(direction);
                    // Also add immediate neighbours if they are traversable 
                    if (neighbours[(direction + 7) % 8].IsTraversable()) 
                        tilesReachableViaCenter.Add((direction + 7) % 8);
                    if (neighbours[(direction + 1) % 8].IsTraversable())
                        tilesReachableViaCenter.Add((direction + 1) % 8);
                }
            }
            
            // If 7 (or 8) of the 8 neighbours are traversable then all traversable
            // tiles are reachable without going through the center 
            if (tilesReachableViaCenter.Count >= 7) return false;
            
            // Otherwise assert that all these are directly reachable via each other
            var firstNeighbour = tilesReachableViaCenter.First();
            var tilesReachableFromFirstNeighbour = new HashSet<int>() {firstNeighbour};
            
            // Add all neighbours reachable by going counter clockwise:
            var tempIndex = firstNeighbour;
            do {
                tilesReachableFromFirstNeighbour.Add(tempIndex);
                tempIndex = (tempIndex + 7) % 8; // Equivalent to subtracting 1, but avoids going into negatives
            } while (neighbours[tempIndex].IsTraversable() && tempIndex != firstNeighbour);
            
            // Add all neighbours reachable by going clockwise:
            tempIndex = firstNeighbour;
            do {
                tilesReachableFromFirstNeighbour.Add(tempIndex);
                tempIndex = (tempIndex + 1) % 8;
            } while (neighbours[tempIndex].IsTraversable() && tempIndex != firstNeighbour);

            // The center blocks a path between two neighbours if the amount of neighbours reachable from the first
            // neighbour is not the same as the number of tiles reachable via the center
            var centerBlocksPath = tilesReachableFromFirstNeighbour.Count != tilesReachableViaCenter.Count;
            return centerBlocksPath;
        }

        // Finds all neighbours of the given tag
        private NeighbourTile[] GetNeighbours(BrickAndMortarTag centerTag) {
            var allTags = GetNearbyTags();

            // Filter out the current tag to get a list of all surrounding tags
            var nearbyTags = 
                allTags
                .Where(relativePos => relativePos.Item.ID != centerTag.ID)
                .ToList();

            var registeredNeighbourIds = centerTag.NeighbourIds;
            NeighbourTile[] neighbours = new NeighbourTile[CardinalDirectionsCount];
            for (int currentDir = 0; currentDir < CardinalDirectionsCount; currentDir++) {
                var neighbourID = registeredNeighbourIds[currentDir];
                // Ignore unknown neighbours
                if (neighbourID != BrickAndMortarTag.UnknownNeighbour) {
                    var tag = nearbyTags.First(item => item.Item.ID == neighbourID)!;
                    neighbours[currentDir] = new NeighbourTile(this, tag.Item.Status, currentDir, centerTag, tag.Item);
                } else {
                    // No registered tags. Detect if the neighbour tile is traversable or solid
                    var neighbourAngle = DirectionToAngle(currentDir);
                    var possibleWall = _controller.DetectWall(neighbourAngle);
                    var maximumWallDistance = IsDirectionDiagonal(currentDir)
                        ? _maximumDiagonalDistance
                        : _maximumAxialDistance;
                    
                    if (possibleWall != null && possibleWall.Value.distance <= maximumWallDistance) {
                        // This tile is not traversable, mark it as solid
                        neighbours[currentDir] = new NeighbourTile(this, TileStatus.Solid, currentDir, centerTag, null);
                    } else {
                        // This tile is not solid, consider it unexplored. 
                        neighbours[currentDir] = new NeighbourTile(this, TileStatus.Unexplored, currentDir, centerTag, null);
                    }
                }
            }
            return neighbours;
        }

        private List<RelativePosition<BrickAndMortarTag>> GetNearbyTags() {
            return _controller
                .ReadNearbyTags()
                // Cast ITag to BrickAndMortarTag
                .Select(item => item.Map((tag) => (BrickAndMortarTag) tag))
                .ToList();
        }

        // Returns the position of the neighbour relative to the robot
        // center is the position of the center tile relative to the robot
        // neighbour is the position of the neighbour relative to the center tile
        public RelativePosition<T> GetNeighbourPosRelativeToRobot<T>(RelativePosition<T> center, RelativePosition<T> neighbour, float globalRobotAngle) {
            var robotAngleVector = new Vector2(Mathf.Cos(globalRobotAngle * Mathf.Deg2Rad), Mathf.Sin(globalRobotAngle * Mathf.Deg2Rad));
            var absoluteAngleToCenter = (globalRobotAngle + center.RelativeAngle) % 360;
            var robotToCenter = new Vector2(Mathf.Cos(absoluteAngleToCenter * Mathf.Deg2Rad), Mathf.Sin(absoluteAngleToCenter * Mathf.Deg2Rad)) * center.Distance;
            var centerToNeighbour = new Vector2(Mathf.Cos(neighbour.RelativeAngle * Mathf.Deg2Rad), Mathf.Sin(neighbour.RelativeAngle * Mathf.Deg2Rad)) * neighbour.Distance;
            var robotToNeighbour = robotToCenter + centerToNeighbour;
            
            // Find angle of neighbour relative to the robot
            var angleRelativeToRobot = Vector2.SignedAngle(robotAngleVector, robotToNeighbour);
            var distanceFromRobot = robotToNeighbour.magnitude;
            return new RelativePosition<T>(distanceFromRobot, angleRelativeToRobot, neighbour.Item);
        } 

        // Deposit new tag
        private BrickAndMortarTag DepositNewTag() {
            var newTag = new BrickAndMortarTag(TileStatus.Explored, _tagIdGenerator.Next());
            _controller.DepositTag(newTag);
            return newTag;
        }

        private void MoveToTag<T>(RelativePosition<T> targetRelativePosition) {
            // First rotate to correctly face the target
            if (Mathf.Abs(targetRelativePosition.RelativeAngle) > 0.5f) {
                _controller.Rotate(targetRelativePosition.RelativeAngle);
                return;
            }
            
            // If already rotated start moving towards the target
            _controller.Move(targetRelativePosition.Distance);
        }

        // Finds the nearest tag that is either unexplored or explored 
        private RelativePosition<BrickAndMortarTag> FindNearestNonSolidTag(BrickAndMortarTag currentTag) {
            var tags = _controller
                .ReadNearbyTags()
                .Select(item => item.Map((tag) => (BrickAndMortarTag) tag));

            RelativePosition<BrickAndMortarTag>? bestCandidate = null;
            foreach (var currentCandidate in tags) {
                if (bestCandidate == null || currentCandidate.Distance < bestCandidate.Distance) 
                    bestCandidate = currentCandidate;
            }
            
            return bestCandidate;
        }
        
        // Updates the given tag (and its neighbours) to have correct neighbour ids
        // This method assumes that the current tag 
        private void UpdateTagNeighbours(NeighbourTile newTile, BrickAndMortarTag lastTag) {
            // TODO: Look for new nearby tags and merge grid! (Our main contribution)
            // TODO: Also merge relevant neighbours of previous tile
            
            var newTag = newTile.Tag!;
            // Update the old tag to have the new tag as neighbour
            lastTag.NeighbourIds[newTile.DirectionFromCenter] = newTag.ID;
            // The new tile must have the old tile as the neighbour in the opposite direction
            // ie. if the new tag is the WEST neighbour of the old tile,
            // then the old tag is the EAST neighbour of the new tag
            newTag.NeighbourIds[OppositeDirection(newTile.DirectionFromCenter)] = lastTag.ID;

            var otherTags = GetNearbyTags()
                .Where(relativeTag => relativeTag.Item.ID != newTag.ID)
                .ToList();
            // Attempt to identify all unknown neighbours
            for (int direction = 0; direction < CardinalDirectionsCount; direction++) {
                if (newTag.NeighbourIds[direction] == BrickAndMortarTag.UnknownNeighbour) {
                    var neighbour = IdentifyNeighbour(otherTags, direction);
                    if (neighbour != null) {
                        newTag.NeighbourIds[direction] = neighbour.ID;
                        neighbour.NeighbourIds[OppositeDirection(direction)] = newTag.ID;
                    }
                }
            }
        }

        // Looks for a tag that is within the given neighbour tile region(relative to the current position of the robot)
        private BrickAndMortarTag? IdentifyNeighbour(List<RelativePosition<BrickAndMortarTag>> relativeTags, int cardinalDirection) {
            bool isDiagonal = IsDirectionDiagonal(cardinalDirection);
            float maximumDistance = isDiagonal ? _maximumDiagonalDistance : _maximumAxialDistance;
            float minimumDistance = isDiagonal ? _minimumDiagonalDistance : _minimumAxialDistance;
            float globalNeighbourAngle = DirectionToAngle(cardinalDirection);
            float robotAngle = this._controller.GetGlobalAngle();

            // Create a bounding box for the neighbour region
            var minimumVector = Geometry.VectorFromDegreesAndMagnitude(globalNeighbourAngle,minimumDistance);
            var maximumVector = Geometry.VectorFromDegreesAndMagnitude(globalNeighbourAngle,maximumDistance);
            var tileSize = Mathf.Max(Mathf.Abs(maximumVector.x - minimumVector.x), Mathf.Abs(maximumVector.y - minimumVector.y));
            var bounds = new Bounds((minimumVector + maximumVector) / 2f, new Vector2(tileSize, tileSize));
            
            // Check if any of the given tags are within the neighbour bounding box 
            foreach (var relativeTag in relativeTags) {
                var globalTagAngle = robotAngle + relativeTag.RelativeAngle;
                var tagVector = Geometry.VectorFromDegreesAndMagnitude(globalTagAngle, relativeTag.Distance);
                if (bounds.Contains(tagVector))
                    return relativeTag.Item;
            }

            // No neighbour was found
            return null;
        }
        
        public void SetController(Robot2DController controller) {
            this._controller = controller;
            this._robotID = _controller.GetRobotID();
            for (int i = 0; i < _directionPriority.Length; i++)
                _directionPriority[i] = (_directionPriority[i] + controller.GetRobotID() * 2) % 8;
        }

        public string GetDebugInfo() {
            return "";
        }
        
        public object SaveState() {
            throw new System.NotImplementedException();
        }

        public void RestoreState(object stateInfo) {
            throw new System.NotImplementedException();
        }
    }
}