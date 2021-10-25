using System;

namespace Dora
{
    public readonly struct RobotConstraints
    {
        // Broadcasting
        public readonly float BroadcastRange;
        public readonly bool BroadcastBlockedByWalls;
        
        // TODO: Movement imprecision
        //public readonly float MaxMovementDeviation;
        // Should probably use some distribution to provide a random distance based on the MaxMovementDeviation
        //public float NextMovementImprecision(Random random, float targetDistance) { throw new NotImplementedException(); }
        
        // TODO: SLAM imprecision
        
        
        public RobotConstraints(float broadcastRange, bool broadcastBlockedByWalls)
        {
            BroadcastRange = broadcastRange;
            BroadcastBlockedByWalls = broadcastBlockedByWalls;
        }
    }
}