using System;

namespace Maes.Robot {
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
        
        // Movement
        // 1.0f is default. A bigger map with bigger doors would make the robot "feel" slower. It is thus not 
        // a speed value in e.g. km/h .
        public readonly float RelativeMoveSpeed;


        public readonly float RobotRelativeSize;
        
        // TODO: Add robot size to constraints class


        public RobotConstraints(float broadcastRange, bool broadcastBlockedByWalls, float senseNearbyRobotRange, bool senseNearbyRobotBlockedByWalls, bool automaticallyUpdateSlam, int slamUpdateIntervalInTicks, int slamSynchronizeIntervalInTicks, float slamPositionInaccuracy, bool distributeSlam, float environmentTagReadRange, float lidarRange, float relativeMoveSpeed, float robotRelativeSize) : this() {
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
            RelativeMoveSpeed = relativeMoveSpeed;
            RobotRelativeSize = robotRelativeSize;

            MaxRayCastRange = Math.Max(SenseNearbyRobotRange, BroadcastRange);
        }

    }
}