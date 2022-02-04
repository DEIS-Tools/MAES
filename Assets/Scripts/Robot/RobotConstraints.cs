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


        // *** Ray cast range and count affects both exploration progress and the slam map ***

        // The amount of traces shot out, higher trace count will result in a more complete map
        // The smaller the ray cast range the lower amount of traces is needed
        // If ray cast count is left unspecified, a default amount will be calculated from the ray cast rang
        public readonly int? SlamRayTraceCount;

        // SLAM ray trace range
        public readonly float SlamRayTraceRange;

        // Environment tagging
        public readonly float EnvironmentTagReadRange;


        // TODO: Add robot size to constraints class


        public RobotConstraints(float broadcastRange, bool broadcastBlockedByWalls, float senseNearbyRobotRange,
            bool senseNearbyRobotBlockedByWalls, bool automaticallyUpdateSlam, int slamUpdateIntervalInTicks,
            int slamSynchronizeIntervalInTicks, float slamPositionInaccuracy, bool distributeSlam,
            float environmentTagReadRange, float slamRayTraceRange,
            int? slamRayTraceCount = null) : this() {
            SlamRayTraceCount = slamRayTraceCount;
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
            SlamRayTraceRange = slamRayTraceRange;
        }
    }
}