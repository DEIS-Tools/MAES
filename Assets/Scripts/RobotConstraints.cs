using System;

namespace Maes {
    public readonly struct RobotConstraints {
        // Broadcasting
        public readonly float BroadcastRange;
        public readonly bool BroadcastBlockedByWalls;

        public readonly float SenseNearbyRobotRange;
        public readonly bool SenseNearbyRobotBlockedByWalls;
        
        // SLAM
        public readonly bool AutomaticallyUpdateSlam;
        public readonly int SlamUpdateIntervalInTicks;
        public readonly int SlamSynchronizeIntervalInTicks;
        public readonly float SlamPositionInaccuracy;
        public readonly bool DistributeSlam;
        
        
        // Environment tagging
        public readonly float EnvironmentTagReadRange;

        public readonly float MaxRayCastRange;

        public readonly float LidarRange;


        public RobotConstraints(float broadcastRange, bool broadcastBlockedByWalls, float senseNearbyRobotRange, bool senseNearbyRobotBlockedByWalls, bool automaticallyUpdateSlam, int slamUpdateIntervalInTicks, int slamSynchronizeIntervalInTicks, float slamPositionInaccuracy, bool distributeSlam, float environmentTagReadRange, float lidarRange) : this() {
            BroadcastRange = broadcastRange;
            BroadcastBlockedByWalls = broadcastBlockedByWalls;
            SenseNearbyRobotRange = senseNearbyRobotRange;
            SenseNearbyRobotBlockedByWalls = senseNearbyRobotBlockedByWalls;
            
            AutomaticallyUpdateSlam = automaticallyUpdateSlam;
            SlamUpdateIntervalInTicks = slamUpdateIntervalInTicks;
            SlamSynchronizeIntervalInTicks = slamSynchronizeIntervalInTicks;
            SlamPositionInaccuracy = slamPositionInaccuracy;
            DistributeSlam = distributeSlam;
            
            EnvironmentTagReadRange = environmentTagReadRange;
            LidarRange = lidarRange;

            MaxRayCastRange = Math.Max(SenseNearbyRobotRange, BroadcastRange);
        }

    }
}