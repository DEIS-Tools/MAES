namespace Dora.Robot {
    public class RelativePosition {
        public readonly float Distance;
        public readonly float RelativeAngle;

        public RelativePosition(float distance, float relativeAngle) {
            Distance = distance;
            RelativeAngle = relativeAngle;
        }
    }
}