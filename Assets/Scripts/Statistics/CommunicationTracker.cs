using System.Collections.Generic;
using Maes.Robot;
using static Maes.Robot.CommunicationManager;
using static Maes.Statistics.ExplorationTracker;

namespace Maes.Statistics {
    public class CommunicationTracker {
        public readonly Dictionary<int, SnapShot<bool>> InterconnectionSnapShot = new Dictionary<int, SnapShot<bool>>();
        private Dictionary<(int, int), CommunicationInfo> _adjacencyMatrixRef;
        private RobotConstraints _robotConstraints;

        public CommunicationTracker(Dictionary<(int, int), CommunicationInfo> adjacencyMatrixRef, RobotConstraints constraints) {
            _adjacencyMatrixRef = adjacencyMatrixRef;
            _robotConstraints = constraints;
        }

        public void CreateSnapshot(int tick) {
            if (AreAllAgentsConnected())
                InterconnectionSnapShot[tick] = new SnapShot<bool>(tick, true);
            else
                InterconnectionSnapShot[tick] = new SnapShot<bool>(tick, false);
        }

        private bool AreAllAgentsConnected() {
            foreach (var comInfo in _adjacencyMatrixRef.Values) {
                // Are robots within communication range?
                if (comInfo.Distance > _robotConstraints.BroadcastRange)
                    // Are robots within line of sight?
                    if (_robotConstraints.BroadcastBlockedByWalls && comInfo.WallsCellsPassedThrough > 0)
                        return false;
            }

            return true;
        }
    }
}