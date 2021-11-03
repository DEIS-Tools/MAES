using System;

namespace Dora.Robot {
    public class RelativePosition<T> {

        public readonly float Distance;
        public readonly float RelativeAngle;
        public readonly T Item;

        public RelativePosition(float distance, float relativeAngle, T item) {
            Distance = distance;
            RelativeAngle = relativeAngle;
            Item = item;
        }

        public RelativePosition<V> Map<V>(Func<T, V> transformer) {
            return new RelativePosition<V>(Distance, RelativeAngle, transformer(Item));
        }
    }
}