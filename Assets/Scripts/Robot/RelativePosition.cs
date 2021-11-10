namespace Dora.Robot {
    public class RelativePosition {
        public readonly float Distance;
        public readonly float RelativeAngle;

        public RelativePosition(float distance, float relativeAngle) {
            Distance = distance;
            RelativeAngle = relativeAngle;
        }

        public override string ToString() {
            return $"[Relative angle: {RelativeAngle} degrees. Distance: {Distance}]";
        }
    }
}