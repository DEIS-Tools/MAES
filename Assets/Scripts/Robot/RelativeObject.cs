using System;

namespace Maes.Robot {
    // Represents a relative position to some object of type T
    public class RelativeObject<T> {

        public readonly float Distance;
        public readonly float RelativeAngle;
        public T Item;

        public RelativeObject(float distance, float relativeAngle, T item) {
            Distance = distance;
            RelativeAngle = relativeAngle;
            Item = item;
        }

        public RelativeObject<V> Map<V>(Func<T, V> transformer) {
            return new RelativeObject<V>(Distance, RelativeAngle, transformer(Item));
        }
    }
}