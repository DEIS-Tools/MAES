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

        // TODO Determine architecture for planning a move (find target -> go there -> exec func -> 
        private BrickAndMortarTag? _lastTag;
        private BrickAndMortarTag? _targetTag;
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
            public RelativePosition<BrickAndMortarTag>? Tag;
            public int Direction;

            public NeighbourTile(TileStatus status, RelativePosition<BrickAndMortarTag>? tag, int direction) {
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
            _maximumAxialDistance = Mathf.Sqrt(Mathf.Pow(_maximumAxialDistance, 2f) / 2f);
            
            _preferredAxialDistance = _maximumAxialDistance / 2f;
        }

        public void UpdateLogic() {
            if (_controller.GetStatus() != RobotStatus.Idle)
                return;
            
            // Initialize algorithm by dropping tag at current position
            if (_lastTag == null) {
                _lastTag = CreateNextTag();
                _controller.DepositTag(CreateNextTag());
            }

            var nextTarget = 
            return;
        }

        private RelativePosition<BrickAndMortarTag?> ChooseNextTile(RelativePosition<BrickAndMortarTag> currentTag) {
            var neighbours = GetNeighbours(currentTag);
            var firstUnexplored = neighbours.First(tile => tile.Status == TileStatus.Unexplored);
            
            return new RelativePosition<BrickAndMortarTag?>(_preferredAxialDistance, DirectionToAngle(firstUnexplored.Direction), null);
        }

        // Finds all neighbours of the given tag
        private NeighbourTile[] GetNeighbours(RelativePosition<BrickAndMortarTag> currentTag) {
            var nearbyTags = _controller
                .ReadNearbyTags()
                // Cast ITag to BrickAndMortarTag
                .Select(item => item.Map((tag) => (BrickAndMortarTag) tag))
                // Remove the current tag from the list
                .Where(relativePos => relativePos.Item.ID != currentTag.Item.ID)
                .ToList();

            var registeredNeighbourIds = currentTag.Item.NeighbourIds;
            NeighbourTile[] neighbours = new NeighbourTile[CardinalDirectionsCount];
            for (int currentDir = 0; currentDir < CardinalDirectionsCount; currentDir++) {
                var neighbourID = registeredNeighbourIds[currentDir];
                // Ignore unknown neighbours
                if (neighbourID != BrickAndMortarTag.UnknownNeighbour) {
                    var tag = nearbyTags.First(item => item.Item.ID == neighbourID)!;
                    neighbours[currentDir] = new NeighbourTile(tag.Item.Status, tag, currentDir);
                } else {
                    // No registered tags. Detect if the neighbour tile is traversable or solid
                    var globalAngle = DirectionToAngle(currentDir);
                    var possibleWall = _controller.DetectWall(globalAngle);
                    var maximumWallDistance = IsDirectionDiagonal(currentDir)
                        ? _maximumDiagonalDistance
                        : _maximumAxialDistance;
                    
                    if (possibleWall != null && possibleWall.Value.distance <= maximumWallDistance) {
                        // This tile is not traversable, mark it as solid
                        neighbours[currentDir] = new NeighbourTile(TileStatus.Solid,null, currentDir);
                    } else {
                        // This tile is not solid, consider it unexplored
                        neighbours[currentDir] = new NeighbourTile(TileStatus.Unexplored,null, currentDir);
                    }
                }
            }
            return neighbours;
        }

        private BrickAndMortarTag CreateNextTag() {
            return new BrickAndMortarTag(TileStatus.Explored, _tagIdGenerator.Next());
        }

        private void MoveToTag<T>(RelativePosition<T> targetRelativePosition) {
            // First rotate to correctly face the target
            if (targetRelativePosition.RelativeAngle > 0.1f) {
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