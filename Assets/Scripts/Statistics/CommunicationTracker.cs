using System;
using System.Collections.Generic;
using System.Linq;
using Maes.Robot;
using static Maes.Robot.CommunicationManager;
using static Maes.Statistics.ExplorationTracker;

namespace Maes.Statistics {
    public class CommunicationTracker {
        public readonly Dictionary<int, SnapShot<bool>> InterconnectionSnapShot = new Dictionary<int, SnapShot<bool>>();
        public readonly Dictionary<int, SnapShot<float>> BiggestClusterPercentageSnapshots = new Dictionary<int, SnapShot<float>>();
        public Dictionary<(int, int), CommunicationInfo> AdjacencyMatrixRef;
        public List<HashSet<int>> CommunicationGroups = null; 
        private RobotConstraints _robotConstraints;

        public CommunicationTracker(RobotConstraints constraints) {
            _robotConstraints = constraints;
        }

        public void CreateSnapshot(int tick) {
            if (tick == 0) return;
            CreateInterconnectedSnapShot(tick);
            CreateClusterSizeSnapShot(tick);
        }

        private void CreateClusterSizeSnapShot(int tick) {
            if (CommunicationGroups != null) {
                // if we have exactly one group, then every agent must be in it!
                if (CommunicationGroups.Count == 1) {
                    BiggestClusterPercentageSnapshots[tick] = new SnapShot<float>(tick, 100.0f);
                }
                else {
                    // Supposed to sort descending
                    CommunicationGroups.Sort((e1, e2) => {
                        return e2.Count.CompareTo(e1.Count);
                    });
                    var totalRobots = CommunicationGroups.Aggregate(0, (sum, e1) => {
                        return sum + e1.Count;
                    });
                    float percentage = (float)CommunicationGroups[0].Count / (float)totalRobots * (float)100;
                    BiggestClusterPercentageSnapshots[tick] = new SnapShot<float>(tick, percentage);
                }
            }
        }

        private void CreateInterconnectedSnapShot(int tick) {
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