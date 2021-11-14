#nullable enable
using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using Dora.MapGeneration;
using Dora.MapGeneration.PathFinding;
using Dora.Robot;
using Dora.Utilities;
using UnityEngine;
using UnityEngine.EventSystems;
using static Dora.MapGeneration.CardinalDirection;
using static Dora.MapGeneration.CardinalDirection.RelativeDirection;

namespace Dora.ExplorationAlgorithm.SSB {
    public class SsbAlgorithm : IExplorationAlgorithm {

        private IRobotController _controller;
        private RobotConstraints _constraints;
        private CoarseGrainedMap _navigationMap;
        private int _randomSeed;

        private State _currentState = State.Backtracking;

      
        
        
        // Backtracking variables
        private Vector2Int? _backtrackTarget;
        private Queue<Vector2Int>? _backtrackingPath;
        private Vector2Int? _nextBackTrackStep;
        private HashSet<Vector2Int> _backTrackingPoints = new HashSet<Vector2Int>();

        // Spiraling information
        // The side that the outer wall of the spiral is on, relative to the spiraling robot
        private RelativeDirection _referenceLateralSide;
        // The opposite side of the rls. This is the side pointing toward the center of the spiral
        private RelativeDirection _oppositeLateralSide;
        // Target for next step in spiral movement
        private Vector2Int? _nextSpiralTarget;
        
        // Primitive time tracking mechanism
        private int _currentTick = 0;
        
        // State variable for tracking the last time that this robot requested or received
        // a list of bp's from other robots
        // This is used to avoid sending requests too often
        private int _lastBPRequestTick = -1;
        private const int MinimumTicksBetweenBpRequests = 10;
        // This variable traces whether this robot has sent its own request the previous tick
        // (to avoid duplicates when multiple robots send a request at the same time)
        private int _tickOfLastRequestSentByThisRobot = int.MinValue;

        private enum State {
            Spiraling,
            Backtracking,
            Waiting,
            Terminated
        }

        private class TileData {
            public bool IsExplored = false;

            public TileData(bool isExplored) {
                IsExplored = isExplored;
            }
        }

        public SsbAlgorithm(RobotConstraints constraints, int randomSeed) {
            _constraints = constraints;
            _randomSeed = randomSeed;
        }

        public void UpdateLogic() {
            _currentTick++;
            // If waiting mode was engaged last tick, then switch back to backtracking mode and attempt to find a bp
            if (_currentState == State.Waiting)  
                _currentState = State.Backtracking;
            
            ProcessIncomingCommunication();
            while (_controller.GetStatus() == RobotStatus.Idle && _currentState != State.Terminated && _currentState != State.Waiting) {
                if (_currentState == State.Backtracking) 
                    PerformBackTrack();
            
                if (_currentState == State.Spiraling)
                    PerformSpiraling();
            }
        }

        private void ProcessIncomingCommunication() {
            var messages = _controller.ReceiveBroadcast();
            var ssbMessages = new List<ISsbBroadcastMessage>();
            foreach (var msg in messages) {
                if (msg is ISsbBroadcastMessage ssbMsg) {
                    CombineOrAdd(ssbMessages, ssbMsg);
                } else {
                    throw new Exception("Received message that was not of type SsbBroadcastMessage");
                }
            }

            // Now process each message
            foreach (var msg in ssbMessages) {
                var response = msg.Process(this);
                // If processing of the message prompts a response then broadcast it 
                if (response != null)
                    _controller.Broadcast(response);
            }
        }

        // Combines received messages when possible
        private void CombineOrAdd(List<ISsbBroadcastMessage> broadcastMessages, ISsbBroadcastMessage newMsg) {
            ISsbBroadcastMessage? overwrittenMessage = null;
            ISsbBroadcastMessage? combinedMsg = null;
            foreach (var existingMessage in broadcastMessages) {
                combinedMsg = existingMessage.Combine(newMsg, this);
                if (combinedMsg != null) {
                    overwrittenMessage = existingMessage;
                    break;
                }
            }

            // If combination was successful then replace old msg with new combined one
            if (combinedMsg != null) {
                broadcastMessages[broadcastMessages.IndexOf(overwrittenMessage!)] = combinedMsg;
            } else {
                // Otherwise just add the new one to the list
                broadcastMessages.Add(newMsg);
            }
        }

        // --------------  Backtracking phase  -------------------
        private void PerformBackTrack() {
            if (_backtrackTarget == null) {
                // Send a request to receives bps from others and start an auction
                // (if enough time has passed since last request)
                if (_currentTick - _lastBPRequestTick >= MinimumTicksBetweenBpRequests) {
                    BroadcastBPRequest();
                    _currentState = State.Waiting;
                }
                
                if (_tickOfLastRequestSentByThisRobot == _currentTick - 2) {
                    Debug.Log("No bids were won by this robot in the auction. Overriding result with target from " +
                              "local list of bps. This may result in conflicts with other robots");
                    // Case of no response from other robots after starting request/auction
                    // or if no bps were won by this robot
                    _backtrackTarget = FindBestBackTrackingTarget();
                    // If no local backtracking points could be found either wait till next tick
                    if (_backtrackTarget == null)
                        _currentState = State.Waiting;
                } else {
                    _currentState = State.Waiting;
                }

                return;
            }
            
            // If no backtracking targets exist, then exploration must have been completed
            if (_backtrackTarget == null) { // TODO : New termination criteria?
                _currentState = State.Waiting;
                return;
            }
            
            // Find path to target, if no path has been found previously
            _backtrackingPath ??= new Queue<Vector2Int>(_navigationMap.GetPath(_backtrackTarget!.Value));
            
            // Check if the path has been completed (ie. no more steps remain)
            if (_nextBackTrackStep == null && _backtrackingPath.Count == 0) {
                // Robot has reached target, ensure that we are oriented at some angle
                // that is aligned with the grid before moving on to the spiral phase
                var isAligned = EnsureCorrectOrientationForSpiraling();
                if (isAligned) {
                    // Ready to enter Spiraling phase. Clear back tracking information
                    _currentState = State.Spiraling;
                    _nextBackTrackStep = null;
                    _backtrackingPath = null;
                    _backtrackTarget = null;
                }
                return;
            }
            
            _nextBackTrackStep ??= _backtrackingPath!.Dequeue();
            
            // Check if the robot needs to move to reach next step target 
            var relativeTarget = _navigationMap.GetTileCenterRelativePosition(_nextBackTrackStep!.Value);
            if (relativeTarget.Distance > 0.2f) 
                MoveTo(relativeTarget);
            else {
                // This tile has been reached, mark it as explored
                MarkTileExplored(_nextBackTrackStep!.Value);
                _nextBackTrackStep = null;
            }
        }

        private void BroadcastBPRequest() {
            Debug.Log($"Backtracking request for backtracking points [Robot: {_controller.GetRobotID()}]");
            _lastBPRequestTick = _currentTick;
            var request = new RequestMessage(_controller.GetRobotID(), this);
            _controller.Broadcast(request);
        }

        // --------------  Spiraling phase  -------------------
        private void PerformSpiraling() {
            _nextSpiralTarget ??= DetermineNextSpiralTarget();

            if (_nextSpiralTarget == null) {
                _currentState = State.Backtracking;
                return;
            }
                
            var relativePosition = _navigationMap.GetTileCenterRelativePosition(_nextSpiralTarget!.Value);
            if (relativePosition.Distance > 0.3f) {
                if (Mathf.Abs(relativePosition.RelativeAngle) > 0.5f) {
                    _controller.Rotate(relativePosition.RelativeAngle);
                } else {
                    // Once facing the tile correctly, add suitable neighbours
                    // as back tracking points for use in next phase
                    DetectBacktrackingPoints();
                    // Finally move towards the target
                    _controller.Move(relativePosition.Distance);
                }
            } else {
                MarkTileExplored(_nextSpiralTarget!.Value);
                _nextSpiralTarget = null;
            }
        }

        private void MarkTileExplored(Vector2Int exploredTile) {
            _navigationMap.SetTileExplored(exploredTile, true);
            // Remove this from possible back tracking candidates, if present
            _backTrackingPoints.Remove(exploredTile);
        }

        // Rotates the robot such that it directly faces a non-solid tile
        private bool EnsureCorrectOrientationForSpiraling() {
            CardinalDirection? targetDirection = null;
            var directions = new List<CardinalDirection>() {East, South, West, North};
            foreach (var direction in directions) {
                if (IsTileBlocked(_navigationMap.GetGlobalNeighbour(direction)))
                    continue;
                
                // If this tile is open and the left neighbour is blocked, start spiraling along the left wall 
                if (IsTileBlocked(_navigationMap.GetGlobalNeighbour(direction.GetRelativeDirection(Left)))) {
                    _referenceLateralSide = Left;
                    _oppositeLateralSide = Right;
                    targetDirection = direction;
                    break;
                }

                // If this tile is open and the right neighbour is blocked, start spiraling along the right wall
                if (IsTileBlocked(_navigationMap.GetGlobalNeighbour(direction.GetRelativeDirection(Right)))) {
                    _referenceLateralSide = Right;
                    _oppositeLateralSide = Left;
                    targetDirection = direction;
                    break;
                }
            }

            if (targetDirection == null) {
                // No surrounding tile is solid, start spiral progress from this point with the following default setup:
                targetDirection = North;
                _referenceLateralSide = Right;
                _oppositeLateralSide = Left;
            }

            // Find relative position of neighbour located in target direction
            var targetRelativePosition = _navigationMap
                .GetTileCenterRelativePosition(_navigationMap.GetGlobalNeighbour(targetDirection));
            // Assert that we are pointed in the right direction
            if (Mathf.Abs(targetRelativePosition.RelativeAngle) <= 1.0f) 
                return true;
            
            // otherwise rotate to face the target tile
            _controller.Rotate(targetRelativePosition.RelativeAngle);
            return false;
        }

        private bool IsTileBlocked(Vector2Int tileCoord) {
            if (_navigationMap.GetTileStatus(tileCoord) == SlamMap.SlamTileStatus.Solid)
                return true; // physically blocked

            // Return virtual blocked status
            return _navigationMap.IsTileExplored(tileCoord);
        }

        private void MoveTo(RelativePosition relativePosition) {
            if (Mathf.Abs(relativePosition.RelativeAngle) > 0.5f)
                _controller.Rotate(relativePosition.RelativeAngle);
            else {
                _controller.Move(relativePosition.Distance);
            }
        }

        private Vector2Int? FindBestBackTrackingTarget() {
            if (_backTrackingPoints.Count == 0) {
                // Attempt to locate nearby backtrack points
                var currentTile = _navigationMap.GetCurrentTile();
                if (!IsTileBlocked(currentTile))
                    return currentTile;
                else
                    return null;
            }

            var robotPosition = _navigationMap.GetApproximatePosition();

            Vector2Int minDistanceBp = _backTrackingPoints.First();
            var minDist = Vector2.Distance(robotPosition, minDistanceBp);
            foreach (var bp in _backTrackingPoints.Skip(1)) {
                var dist = Vector2.Distance(robotPosition, bp);
                if (dist < minDist) {
                    minDist = dist;
                    minDistanceBp = bp;
                }
            }
            
            _backTrackingPoints.Remove(minDistanceBp);
            return minDistanceBp;
        }

        // Will perform spiral movement according to the algorithm described in the paper
        // Returns true if successful, and false if no free open are available 
        private Vector2Int? DetermineNextSpiralTarget() {
            var front = _navigationMap.GetRelativeNeighbour(Front);
            var rls = _navigationMap.GetRelativeNeighbour(_referenceLateralSide);
            var ols = _navigationMap.GetRelativeNeighbour(_oppositeLateralSide);
            
            var frontTileBlocked = IsTileBlocked(front);
            var rlsBlocked = IsTileBlocked(rls);
            var olsBlocked = IsTileBlocked(ols);

            if (frontTileBlocked && rlsBlocked && olsBlocked)
                return null; // No more open tiles left. Spiraling has finished

            // The following is the spiral algorithm specified by the bsa paper
            if (!rlsBlocked) 
                return _navigationMap.GetRelativeNeighbour(_referenceLateralSide);
            if (frontTileBlocked) 
                return _navigationMap.GetRelativeNeighbour(_oppositeLateralSide);
            return _navigationMap.GetRelativeNeighbour(Front);
        }

        private RelativeDirection? GetSpiralTargetDirection(bool frontBlocked, bool rlsBlocked, bool olsBlocked) {
            if (frontBlocked && rlsBlocked && olsBlocked)
                return null; // No more open tiles left. Spiraling has finished

            // The following is the spiral algorithm specified by the bsa paper
            if (!rlsBlocked)
                return _referenceLateralSide;
            if (frontBlocked) 
                return _oppositeLateralSide;
            return RelativeDirection.Front;
        }

        // Adds backtracking points as specified by the algorithm presented in the SSB paper
        // In the SSB variation of the BP identification, a tile is only considered to be a BP if
        // it has solid tile to its immediate right or left
        private void DetectBacktrackingPoints() {
            if (!IsBlocked(Right)) {
                if (IsBlocked(Front) || IsBlocked(FrontRight) || IsBlocked(RearRight))
                    _backTrackingPoints.Add(_navigationMap.GetRelativeNeighbour(Right));
            }

            if (!IsBlocked(Left)) {
                if (IsBlocked(Front) || IsBlocked(FrontLeft) || IsBlocked(RearLeft))
                    _backTrackingPoints.Add(_navigationMap.GetRelativeNeighbour(Left));
            }
        }

        // Simulates what spiraling might look like assuming all unknown tiles are non-solid
        // Returns the amount of tiles left to traverse
        private int? SimulateSpiraling(int maxCost = 100) {
            if (_currentState != State.Spiraling)
                throw new Exception("Illegal state. Can only call SimulateSpiraling when in spiraling mode");

            // Number of tiles left to traverse
            int cost = 1;

            HashSet<Vector2Int> simulatedExplored = new HashSet<Vector2Int>();

            var simulatedSpiralTile = _nextSpiralTarget;
            // Default to current tile
            simulatedSpiralTile ??= _navigationMap.GetCurrentTile();
            int stepsSinceRotating = 0;
            int maxStepsBeforeRotating = 20;
            
            var simulatedDirection = DirectionFromDegrees(_navigationMap.GetApproximateGlobalDegrees());
            while(cost < maxCost) {
                simulatedExplored.Add(simulatedSpiralTile.Value);
                var simulatedFront = simulatedSpiralTile.Value + simulatedDirection.DirectionVector;
                var simulatedRls = simulatedSpiralTile.Value + simulatedDirection.GetRelativeDirection(_referenceLateralSide).DirectionVector;
                var simulatedOls = simulatedSpiralTile.Value + simulatedDirection.GetRelativeDirection(_oppositeLateralSide).DirectionVector;

                var frontBlocked = !_navigationMap.IsPotentiallyExplorable(simulatedFront) || simulatedExplored.Contains(simulatedFront); 
                var rlsBlocked = !_navigationMap.IsPotentiallyExplorable(simulatedRls)  || simulatedExplored.Contains(simulatedRls); 
                var olsBlocked = !_navigationMap.IsPotentiallyExplorable(simulatedOls)  || simulatedExplored.Contains(simulatedOls);

                var nextDirection = GetSpiralTargetDirection(frontBlocked, rlsBlocked, olsBlocked);
                if (nextDirection == null) // Spiralling successfully terminated
                    return cost;

                if (nextDirection == Front) {
                    stepsSinceRotating++;
                    // If going too far in a straight line, we have probably exited the map
                    // or the spiral is too big to simulate
                    if (stepsSinceRotating > maxStepsBeforeRotating)
                        return null;
                }
                else {
                    stepsSinceRotating = 0;
                }

                // Step the simulation forward to the next tile in the spiral
                simulatedDirection = simulatedDirection.GetRelativeDirection(nextDirection.Value);
                simulatedSpiralTile += simulatedDirection.DirectionVector;
                cost++;
            }
            
            return null;
        }

        private bool IsBlocked(RelativeDirection direction) {
            return IsTileBlocked(_navigationMap.GetRelativeNeighbour(direction));
        }

        public void SetController(Robot2DController controller) {
            this._controller = controller;
            _navigationMap = _controller.GetSlamMap().GetCoarseMap();
        }

        public string GetDebugInfo() {
            return $"State: {Enum.GetName(typeof(State), _currentState)}" +
                   $"\nCoarse Map Position: {_navigationMap.GetApproximatePosition()}" +
                   $"\nTotal BPs: {_backTrackingPoints.Count}";
        }

        public object SaveState() {
            throw new System.NotImplementedException();
        }

        public void RestoreState(object stateInfo) {
            throw new System.NotImplementedException();
        }
        
        // Represents a request to broadcast all available backtracking points found by this robot
        private class RequestMessage: ISsbBroadcastMessage {

            public readonly int RequestingRobot;

            public RequestMessage(int requestingRobot, SsbAlgorithm algorithm) {
                RequestingRobot = requestingRobot;
                algorithm._tickOfLastRequestSentByThisRobot = algorithm._currentTick;
            }

            public ISsbBroadcastMessage? Process(SsbAlgorithm algorithm) {
                // Check for possible conflict
                if (algorithm._tickOfLastRequestSentByThisRobot == algorithm._currentTick - 1) {
                    // This case occurs when this robot has sent a bp request at the same time that another robot has
                    // sent one. Determine which one to discard based on the robots' ids
                    if (this.RequestingRobot < algorithm._controller.GetRobotID()) {
                        // Discard this request received by the other robot, as it has a lower id than this robot 
                        return null;
                    }
                    // If not discarded then continue and create a response for this request
                }
                
                // Remove backtracking points that have been explored since they were added to the list 
                algorithm._backTrackingPoints.RemoveWhere(bp => algorithm._navigationMap.IsTileExplored(bp));
                Debug.Log($"Robot {algorithm._controller.GetRobotID()} received bp request. Broadcasting {algorithm._backTrackingPoints.Count} bps");
                
                if (algorithm._backTrackingPoints.Count == 0) 
                    return null; // No BPs to share

                // Respond to the request by sending all bps found by this robot
                return new BackTrackingPointsMessage(RequestingRobot,new HashSet<Vector2Int>(algorithm._backTrackingPoints));
            }

            public ISsbBroadcastMessage? Combine(ISsbBroadcastMessage other, SsbAlgorithm _) {
                // In case of multiple simultaneous request messages use
                // one the one issued by the robot with the highest id 
                if (other is RequestMessage otherRequestMsg)
                    return otherRequestMsg.RequestingRobot > this.RequestingRobot ? otherRequestMsg : this;

                return null;
            }
        }

        // Represents a message containing all known unexplored bps for an agent
        private class BackTrackingPointsMessage: ISsbBroadcastMessage {

            // The id of the robot that sent the original request for BPs
            public readonly int RequestingRobot;
            
            public readonly HashSet<Vector2Int> BackTrackingPoints;

            public BackTrackingPointsMessage(int requestingRobot, HashSet<Vector2Int> backTrackingPoints) {
                BackTrackingPoints = backTrackingPoints;
                RequestingRobot = requestingRobot;
            }

            // Generates a bid for each bp currently in its list
            public ISsbBroadcastMessage? Process(SsbAlgorithm algorithm) {
                // Add potentially unknown bps from other robots
                algorithm._backTrackingPoints.UnionWith(BackTrackingPoints);

                // If this robot the one that requested the BPs then do not respond to this message
                if (RequestingRobot == algorithm._controller.GetRobotID())
                    return null;

                var bids = algorithm.GenerateBids(algorithm._backTrackingPoints);
                
                Debug.Log($"Robot {algorithm._controller.GetRobotID()} generated {bids.Count} bids");
                
                if (bids.Count == 0)
                    return null;

                return new BiddingMessage(RequestingRobot, bids);
            }
            
            public ISsbBroadcastMessage? Combine(ISsbBroadcastMessage other, SsbAlgorithm _) {
                if (other is BackTrackingPointsMessage bpMessage) {
                    BackTrackingPoints.UnionWith(bpMessage.BackTrackingPoints);
                    return this;
                }

                return null;
            }
        }

        // Generates a series of bids for each of the given backtracking points
        private List<Bid> GenerateBids(HashSet<Vector2Int> backTrackingPoints) {
            List<Bid> bids = new List<Bid>();

            int? spiralFinishCost = 0;
            if (_currentState == State.Spiraling)
                spiralFinishCost = SimulateSpiraling();

            // Spiraling could not be fully simulated
            if(spiralFinishCost == null)
                return null;

            // Generate a bid based on the length of the calculated path to reach the bp (if present)
            foreach (var bp in backTrackingPoints) {
                var potentialPath = _navigationMap.GetPath(bp);
                if (potentialPath != null) {
                    var pathLength = GetRobotPathLength(potentialPath);
                    bids.Add(new Bid(bp, spiralFinishCost.Value + pathLength, _controller.GetRobotID()));
                    Debug.Log($"Robot: {_controller.GetRobotID()} added bid of cost {spiralFinishCost.Value + pathLength} for tile {bp}");
                }
            }
            
            return bids;
        }

        private float GetRobotPathLength(List<Vector2Int> path) {
            var lastTile = _navigationMap.GetCurrentTile();
            var totalDistance = 0f;
            path.ForEach(tile => {
                totalDistance += Vector2Int.Distance(lastTile, tile);
                lastTile = tile;
            });
            return totalDistance;
        }

        private class Bid {
            
            public readonly Vector2Int BP;
            // Lower cost is better
            public readonly float cost;
            public readonly int RobotId;

            public Bid(Vector2Int bp, float cost, int robotId) {
                BP = bp;
                this.cost = cost;
                RobotId = robotId;
            }
        }

        // Represents a series of bids (cost estimation) for each known bp 
        private class BiddingMessage: ISsbBroadcastMessage {

            public readonly int RequestingRobot;
            private readonly Dictionary<Vector2Int, List<Bid>> AllBids = new Dictionary<Vector2Int, List<Bid>>();

            public BiddingMessage(int requestingRobot, List<Bid> robotBids) {
                RequestingRobot = requestingRobot;
                foreach (var robotBid in robotBids) 
                    AllBids.Add(robotBid.BP, new List<Bid>(){robotBid});
            }

            public ISsbBroadcastMessage? Process(SsbAlgorithm algorithm) {
                // This message is ignored if this robot was not the one to start the auction
                if (RequestingRobot != algorithm._controller.GetRobotID())
                    return null;
                
                Debug.Log($"Auction processed by robot: {algorithm._controller.GetRobotID()}");

                // Add bids from this robot to the auction
                foreach (var ownBid in algorithm.GenerateBids(algorithm._backTrackingPoints)) {
                    if(!AllBids.ContainsKey(ownBid.BP)) 
                        AllBids.Add(ownBid.BP, new List<Bid>());
                    
                    AllBids[ownBid.BP].Add(ownBid);
                }

                // Sort each entry by cost (ascending) 
                foreach (var entry in AllBids) {
                    entry.Value.Sort((bid1, bid2) => bid1.cost.CompareTo(bid2.cost));
                }
                
                // Each robot can only win ONE bid so best bids are stored in a dictionary
                var bestBids = new Dictionary<int, Bid>();
                
                // Map dictionary values to queues
                var bidQueues = AllBids.ToDictionary(kvp => kvp.Key, kvp => new Queue<Bid>(kvp.Value));

                var fixedPointReached = false;
                while (!fixedPointReached) {
                    fixedPointReached = true;

                    foreach (var entry in bidQueues) {
                        if (entry.Value.Count == 0)
                            continue; // No more bids left for this tile
                        
                        var newBid = entry.Value.Peek();
                        if (bestBids.ContainsKey(newBid.RobotId)) {
                            // Skip if this is already registered as the current best bid for this robot
                            if (bestBids[newBid.RobotId] == entry.Value.Peek())
                                continue;
                            
                            fixedPointReached = false;
                            if (newBid.cost < bestBids[newBid.RobotId].cost) {
                                // Dequeue previous best value to allow bids from other robot on that tile 
                                bidQueues[bestBids[newBid.RobotId].BP].Dequeue();
                                // Register this as the new best bid for this robot
                                bestBids[newBid.RobotId] = newBid;
                            } else {
                                // Dequeue this new bid, as there exists a better candidate for this robot
                                entry.Value.Dequeue();
                            }
                        } else {
                            bestBids[newBid.RobotId] = newBid;
                            fixedPointReached = false;
                        }
                    }
                }
                
                Debug.Log($"Auction results: {bestBids}");
                
                // Create message containing results
                var resultsMessage = new AuctionResultsMessage(bestBids);
                // Process results for this robot, as it will not receive the message next tick
                resultsMessage.Process(algorithm);
                // Finally broadcast result to all other robots
                return resultsMessage;
            }

            public ISsbBroadcastMessage? Combine(ISsbBroadcastMessage other, SsbAlgorithm algorithm) {
                if (other is BiddingMessage bidMessage) {
                    // This message can be ignored if this robot was not the one to start the auction
                    if (RequestingRobot != algorithm._controller.GetRobotID())
                        return this;
                    
                    // Adds all bids from the other message into this one
                    foreach (var entry in bidMessage.AllBids) {
                        if (!AllBids.ContainsKey(entry.Key)) 
                            AllBids.Add(entry.Key, new List<Bid>());
                        AllBids[entry.Key].AddRange(entry.Value);
                    }

                    return this;
                }
                return null;
            }
        }


        // Represents a broadcasted message containing the winning bids
        private class AuctionResultsMessage : ISsbBroadcastMessage {

            public Dictionary<int, Bid> Results;

            public AuctionResultsMessage(Dictionary<int, Bid> results) {
                Results = results;
            }

            public ISsbBroadcastMessage? Process(SsbAlgorithm algorithm) {
                var robot = algorithm._controller.GetRobotID();
                if (Results.ContainsKey(robot)) {
                    Debug.Log($"Auction resulted in reservation of bp {Results[robot]} for robot {robot}");
                    algorithm._backtrackTarget = Results[robot].BP;
                }
                else {
                    Debug.Log($"Auction resulted in no bp for robot {robot}");
                    algorithm._backtrackTarget = null;
                }
                
                // No further messages needed
                return null;
            }
            
            public ISsbBroadcastMessage Combine(ISsbBroadcastMessage other, SsbAlgorithm _) {
                // There should never be more than one auction result message at a time
                if (other is AuctionResultsMessage)
                    throw new Exception("Illegal state. Multiple auction results received simultaneously");
                return null;
            }
        }
        
    }
}