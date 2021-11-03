#nullable enable
using System;
using System.Linq;
using Dora.MapGeneration;
using Dora.Robot;
using Dora.Robot.Task;
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
        // The maximum distance between along either axis (x or y)
        private readonly float _maximumAxialDistance;

        private readonly float _preferredAxialDistance;
        private readonly float _preferredDiagonalDistance;

        // TODO Determine architecture for planning a move (find target -> go there -> exec func) ... Not flexible (Collisions, other robot places tag etc)
        private BrickAndMortarTag? _lastTag;
        private Action _onTargetReached;
        
        public enum TileStatus {
            // Unexplored indicates that no robot has been here
            // Explored indicated that one or more robots have been here, but may need to traverse it again
            // Visited indicates that this tile is fully explored and need not be visited later (effectively a wall)
            // Solid indicate that the tile is physically in-traversable
            Unexplored = 0, Explored = 1, Visited = 2, Solid = 3
        }

        // Represents an immediate neighbour of a tag 
        private class NeighbourTile {
            public TileStatus Status;
            public RelativePosition<BrickAndMortarTag?>? Tag;
            public int Direction;

            public NeighbourTile(TileStatus status, RelativePosition<BrickAndMortarTag?>? tag, int direction) {
                Status = status;
                Tag = tag;
                Direction = direction;
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
        
        private int OppositeDirection(int direction) => (direction + 4) % 8;
        private int DirectionToAngle(int direction) => ((8 - direction) % 8) * 45;
        private bool IsDirectionDiagonal(int direction) => direction % 2 != 0;
        
        public BrickAndMortar(RobotConstraints constraints, int randomSeed) {
            _constraints = constraints;
            _randomSeed = randomSeed;
            _tagIdGenerator = new Random(randomSeed);
            _maximumDiagonalDistance = constraints.EnvironmentTagReadRange / 2f - TolerablePositioningError;
            // Axial distance formula derived from a² + b² = c²  (And since we operate in a grid we have a = b)
            // This gives   2a² = c²  then rearranging to:  a = sqrt(c²/2)
            _maximumAxialDistance = Mathf.Sqrt(Mathf.Pow(_maximumDiagonalDistance, 2f) / 2f);
            
            _preferredAxialDistance = _maximumAxialDistance / 2f;
            _preferredDiagonalDistance = _maximumDiagonalDistance / 2f;
        }

        public void UpdateLogic() {
            if (_controller.GetStatus() != RobotStatus.Idle)
                return;
            
            // If uninitialized, then init the algorithm by dropping tag at current position
            _lastTag ??= DepositNewTag();

            // Update status of current tile and calculate the best tile to visit next
            var targetTile = ChooseNextTile(_lastTag);

            // If we are not already at this tile, start rotating/moving towards it
            if (targetTile.Tag!.Distance > TolerablePositioningError) {
                MoveToTag(targetTile.Tag);
                return;
            }
            
            // If there is no tag at this tile, ie. it is unexplored, place a new tag and use that instead
            targetTile.Tag.Item ??= DepositNewTag();
            // Update neighbour information for this tag and its neighbours
            UpdateTagNeighbours(targetTile, _lastTag);
            
            // Lastly, update the 'last tag' to be the one we are currently standing on
            _lastTag = targetTile.Tag.Item;
        }

        private NeighbourTile ChooseNextTile(BrickAndMortarTag currentTag) {
            var neighbours = GetNeighbours(currentTag);
            var firstUnexplored = neighbours.First(tile => tile.Status == TileStatus.Unexplored);
            return firstUnexplored;
            //return new RelativePosition<BrickAndMortarTag?>(_preferredAxialDistance, DirectionToAngle(firstUnexplored.Direction), null);
        }

        // Finds all neighbours of the given tag
        private NeighbourTile[] GetNeighbours(BrickAndMortarTag centerTag) {
            var allTags = _controller
                .ReadNearbyTags()
                // Cast ITag to BrickAndMortarTag
                .Select(item => item.Map((tag) => (BrickAndMortarTag) tag))
                .ToList();
            
            // Find the relative position of the current tag
            var centerRelativeToRobot = allTags.First(t => t.Item.ID == centerTag.ID);
            var robotGlobalAngle = _controller.GetGlobalAngle();
            
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
                    neighbours[currentDir] = new NeighbourTile(tag.Item.Status, tag, currentDir);
                } else {
                    // No registered tags. Detect if the neighbour tile is traversable or solid
                    var neighbourAngle = DirectionToAngle(currentDir);
                    var possibleWall = _controller.DetectWall(neighbourAngle);
                    var maximumWallDistance = IsDirectionDiagonal(currentDir)
                        ? _maximumDiagonalDistance
                        : _maximumAxialDistance;
                    
                    if (possibleWall != null && possibleWall.Value.distance <= maximumWallDistance) {
                        // This tile is not traversable, mark it as solid
                        neighbours[currentDir] = new NeighbourTile(TileStatus.Solid,null, currentDir);
                    } else {
                        // This tile is not solid, consider it unexplored. 
                        // Find the position of this tile relative to the robot
                        var neighbourDistance = IsDirectionDiagonal(currentDir)
                            ? _preferredDiagonalDistance
                            : _preferredAxialDistance;
                        var neighbourRelativeToCenter = new RelativePosition<BrickAndMortarTag?>(neighbourDistance, neighbourAngle, null);
                        var neighbourRelativeToRobot = GetNeighbourPosRelativeToRobot(centerRelativeToRobot, neighbourRelativeToCenter, robotGlobalAngle);
                        neighbours[currentDir] = new NeighbourTile(TileStatus.Unexplored, neighbourRelativeToRobot, currentDir);
                    }
                }
            }
            return neighbours;
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
        private void UpdateTagNeighbours(NeighbourTile newTile, BrickAndMortarTag lastTag) {
            // TODO: Look for new nearby tags and merge grid! (Our main contribution)
            
            var newTag = newTile.Tag!.Item!;
            // Update the old tag to have the new tag as neighbour
            lastTag.NeighbourIds[newTile.Direction] = newTag.ID;
            // The new tile must have the old tile as the neighbour in the opposite direction
            // ie. if the new tag is the WEST neighbour of the old tile,
            // then the old tag is the EAST neighbour of the new tag
            newTag.NeighbourIds[OppositeDirection(newTile.Direction)] = lastTag.ID;
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