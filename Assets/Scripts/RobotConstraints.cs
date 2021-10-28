using System;

namespace Dora {
    public readonly struct RobotConstraints {
        // Broadcasting
        public readonly float BroadcastRange;
        public readonly bool BroadcastBlockedByWalls;
        public readonly bool ShouldAutomaticallyUpdateSlam;
        public readonly int SlamUpdateIntervalInTicks;
        public readonly float PositionInaccuracy;

        // TODO: Movement imprecision
        //public readonly float MaxMovementDeviation;
        // Should probably use some distribution to provide a random distance based on the MaxMovementDeviation
        //public float NextMovementImprecision(Random random, float targetDistance) { throw new NotImplementedException(); }

        // TODO: SLAM imprecision

        public RobotConstraints(float broadcastRange, bool broadcastBlockedByWalls, bool shouldAutomaticallyUpdateSlam, int slamUpdateIntervalInTicks, float positionInaccuracy) {
            BroadcastRange = broadcastRange;
            BroadcastBlockedByWalls = broadcastBlockedByWalls;
            ShouldAutomaticallyUpdateSlam = shouldAutomaticallyUpdateSlam;
            SlamUpdateIntervalInTicks = slamUpdateIntervalInTicks;
            PositionInaccuracy = positionInaccuracy;
        }
    }
}