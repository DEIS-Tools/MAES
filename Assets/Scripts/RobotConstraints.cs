using System;

namespace Dora {
    public readonly struct RobotConstraints {
        // Broadcasting
        public readonly float BroadcastRange;
        public readonly bool BroadcastBlockedByWalls;
        
        // SLAM
        public readonly bool ShouldAutomaticallyUpdateSlam;
        public readonly int SlamUpdateIntervalInTicks;
        public readonly float PositionInaccuracy;
        
        // Environment tagging
        public readonly float EnvironmentTagReadRange;

        
        public RobotConstraints(float broadcastRange, bool broadcastBlockedByWalls, bool shouldAutomaticallyUpdateSlam, int slamUpdateIntervalInTicks, float positionInaccuracy, float environmentTagReadRange) {
            BroadcastRange = broadcastRange;
            BroadcastBlockedByWalls = broadcastBlockedByWalls;
            ShouldAutomaticallyUpdateSlam = shouldAutomaticallyUpdateSlam;
            SlamUpdateIntervalInTicks = slamUpdateIntervalInTicks;
            PositionInaccuracy = positionInaccuracy;
            EnvironmentTagReadRange = environmentTagReadRange;
        }
    }
}