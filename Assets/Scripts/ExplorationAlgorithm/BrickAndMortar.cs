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

        // TODO Determine architecture for planning a move (find target -> go there -> exec func) ... Not flexible (Collisions, other robot places tag etc)
        private BrickAndMortarTag? _lastTag;
        private NeighbourTile? _targetTile;
        
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
            _maximumDiagonalDistance = constraints.EnvironmentTagReadRange - TolerablePositioningError - 1f;
            // Axial distance formula derived from a² + b² = c²  (And since we operate in a grid we have a = b)
            // This gives   2a² = c²  then rearranging to:  a = sqrt(c²/2)
            _maximumAxialDistance = Mathf.Sqrt(Mathf.Pow(_maximumDiagonalDistance, 2f) / 2f);

            _preferredDiagonalDistance = _maximumDiagonalDistance * (2f/3f);
            _preferredAxialDistance = _maximumAxialDistance * (2f/3f);

            _minimumDiagonalDistance = _maximumDiagonalDistance * (1f / 3f);
            _minimumAxialDistance = _maximumAxialDistance * (1f / 3f);
        }

        public void UpdateLogic() {
            if (_controller.GetStatus() != RobotStatus.Idle)
                return;
            
            // If uninitialized, then init the algorithm by dropping tag at current position
            _lastTag ??= DepositNewTag();

            // If no target tile is currently chosen, then update the current tile status and find next tile to visit
            _targetTile ??= ChooseNextTile(_lastTag);
            
            
            var targetRelativePosition = _targetTile.GetRelativePosition();

            // If we are not already at this tile, start rotating/moving towards it
            if (targetRelativePosition.Distance > TolerablePositioningError) {
                MoveToTag(targetRelativePosition);
                return;
            }
            
            // If there is no tag at this tile, ie. it is unexplored, place a new tag and use that instead
            _targetTile.Tag ??= DepositNewTag();
            // Update neighbour information for this tag and its neighbours
            UpdateTagNeighbours(_targetTile, _lastTag);
            
            // Lastly, update the 'last tag' to be the one we are currently standing on and clear the target tile
            _lastTag = _targetTile.Tag;
            _targetTile = null;
        }

        private NeighbourTile ChooseNextTile(BrickAndMortarTag currentTag) {
            var neighbours = GetNeighbours(currentTag);

            int bestTileIndex = 0;
            var bestScore = -1;
            for (int i = 0; i < CardinalDirectionsCount; i+=2) {
                // Skip if not traversable (TODO: Also allow explored later)
                if (neighbours[i].Status != TileStatus.Unexplored)
                    continue;
                var score = 0;

                // Check if the two neighbours of this tile are solid/visited, if so increase the score
                var neighbour1 = neighbours[(i + 7) % CardinalDirectionsCount];
                var neighbour2 = neighbours[(i + 1) % CardinalDirectionsCount];
                if (neighbour1.Status == TileStatus.Solid || neighbour1.Status == TileStatus.Visited) score++;
                if (neighbour2.Status == TileStatus.Solid || neighbour2.Status == TileStatus.Visited) score++;

                if (score > bestScore) {
                    bestScore = score;
                    bestTileIndex = i;
                }
            }
            
            return neighbours[bestTileIndex];
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