using System;
using System.Collections.Generic;
using Maes.Robot;
using static Maes.Robot.CommunicationManager;
using static Maes.Statistics.ExplorationTracker;

namespace Maes.Statistics {
    public class CommunicationTracker {
        public readonly Dictionary<int, SnapShot<bool>> InterconnectionSnapShot = new Dictionary<int, SnapShot<bool>>();
        public Dictionary<(int, int), CommunicationInfo> AdjacencyMatrixRef;
        private RobotConstraints _robotConstraints;

        public CommunicationTracker(RobotConstraints constraints) {
            _robotConstraints = constraints;
        }

        public void CreateSnapshot(int tick) {
            if (tick == 0) return;
            if (AdjacencyMatrixRef != null) {
                if (AreAllAgentsConnected())
                    InterconnectionSnapShot[tick] = new SnapShot<bool>(tick, true);
                else
                    InterconnectionSnapShot[tick] = new SnapShot<bool>(tick, false);
            }
        }

        private bool AreAllAgentsConnected() {
            foreach (var comInfo in AdjacencyMatrixRef.Values) {
                // Are robots within communication range?
                if (comInfo.Distance > _robotConstraints.BroadcastRange)
                    return false;
                // Are robots within line of sight?
                if (_robotConstraints.BroadcastBlockedByWalls && comInfo.WallsCellsPassedThrough > 0)
                    return false;
            }
            return true;
        }
    }
}