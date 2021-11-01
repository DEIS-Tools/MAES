using System;

namespace Dora {
    public readonly struct RobotConstraints {
        // Broadcasting
        public readonly float BroadcastRange;
        public readonly bool BroadcastBlockedByWalls;

        public readonly float SenseNearbyRobotRange;
        public readonly bool SenseNearbyRobotBlockedByWalls;
        
        // SLAM
        public readonly bool ShouldAutomaticallyUpdateSlam;
        public readonly int SlamUpdateIntervalInTicks;
        public readonly float PositionInaccuracy;
        
        // Environment tagging
        public readonly float EnvironmentTagReadRange;

        public readonly float MaxRayCastRange;

        
        public RobotConstraints(float broadcastRange, bool broadcastBlockedByWalls, bool shouldAutomaticallyUpdateSlam, int slamUpdateIntervalInTicks, float positionInaccuracy, float environmentTagReadRange, float senseNearbyRobotRange, bool senseNearbyRobotBlockedByWalls) {
            BroadcastRange = broadcastRange;
            BroadcastBlockedByWalls = broadcastBlockedByWalls;
            ShouldAutomaticallyUpdateSlam = shouldAutomaticallyUpdateSlam;
            SlamUpdateIntervalInTicks = slamUpdateIntervalInTicks;
            PositionInaccuracy = positionInaccuracy;
            EnvironmentTagReadRange = environmentTagReadRange;
            SenseNearbyRobotRange = senseNearbyRobotRange;
            SenseNearbyRobotBlockedByWalls = senseNearbyRobotBlockedByWalls;
            
            MaxRayCastRange = Math.Max(SenseNearbyRobotRange, BroadcastRange);
        }
    }
}