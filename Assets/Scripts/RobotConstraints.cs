namespace Dora
{
    public readonly struct RobotConstraints
    {
        public readonly float BroadcastRange;

        public readonly bool BroadcastBlockedByWalls;

        public RobotConstraints(float broadcastRange, bool broadcastBlockedByWalls)
        {
            BroadcastRange = broadcastRange;
            BroadcastBlockedByWalls = broadcastBlockedByWalls;
        }
    }
}