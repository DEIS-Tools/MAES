using System.Collections.Generic;

namespace Dora.Utilities {
    public class Functional {
        public delegate T Factory<T>();

        public delegate bool XBetterThanY<T>(T x, T y);

        public static T TakeBest<T>(T[] values, XBetterThanY<T> xBetterThanY) {
            var currentMin = values[0];
            for (int i = 1; i < values.Length; i++) {
                if (xBetterThanY(values[i], currentMin))
                    currentMin = values[i];
            }
            return currentMin;
        }
        
        
        public static T TakeBest<T>(List<T> values, XBetterThanY<T> xBetterThanY) {
            var currentMin = values[0];
            for (int i = 1; i < values.Count; i++) {
                if (xBetterThanY(values[i], currentMin))
                    currentMin = values[i];
            }
            return currentMin;
        }
    }
}