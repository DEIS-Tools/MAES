using System;
using UnityEngine;

namespace Dora.MapGeneration {
    
    // Represents the 8 directions found on a compass 
    public class CardinalDirection {
        
        public const int CardinalDirectionsCount = 8;
        // Index representing 8 neighbouring tags/tiles
        public static readonly CardinalDirection
            East = new CardinalDirection(0),
            SouthEast = new CardinalDirection(1),
            South = new CardinalDirection(2),
            SouthWest = new CardinalDirection(3),
            West = new CardinalDirection(4),
            NorthWest = new CardinalDirection(5),
            North = new CardinalDirection(6),
            NorthEast = new CardinalDirection(7);

        public enum RelativeDirection {
            // Each relative direction is assign to the corresponding compass offset
            Front = 0, 
            FrontRight = 1, FrontLeft = -1, 
            Left = -2, 
            Right = 2,
            RearRight = 3, RearLeft = -3,
            Rear = 4
        }
        
        private static readonly CardinalDirection[] Directions = 
            {East, SouthEast, South, SouthWest, West, NorthWest, North, NorthEast};
        
        public readonly int Index;
        public readonly Vector2Int Vector;
        
        // Can only be constructed locally. Must be accessed through public static instances
        private CardinalDirection(int index) {
            Index = index;
            Vector = CalculateDirectionVector();
        }

        public CardinalDirection OppositeDirection() => GetDirection((Index + 4) % 8);
        public float DirectionToAngle() => ((8 - Index) % 8) * 45; 
        public bool IsDiagonal() => Index % 2 != 0;

        // Converts the given absolute angle (relative to the x-axis) to the closest corresponding cardinal direction
        public static CardinalDirection DirectionFromDegrees(float degrees) {
            if (degrees < 0f)
                throw new ArgumentException($"Degrees must be above zero, was: {degrees}");

            var offset = (int) (((degrees + 22.5f) % 360) / 45f);
            return Directions[(8 - offset) % 8];
        }

        public static CardinalDirection GetDirection(int index) {
            while (index < 0) index += 8;
            return Directions[index % 8];
        }

        public CardinalDirection Next() {
            return GetDirection(Index + 1);
        }

        public CardinalDirection Previous() {
            return GetDirection(Index - 1);
        }

        private Vector2Int CalculateDirectionVector() {
            var xDir = 0;
            var yDir = 0;
            
            if (Index > 6 || Index < 2) xDir = 1;
            else if (Index < 6 && Index > 2) xDir = -1;
            
            if (Index > 4) yDir = 1;
            else if (Index < 4 && Index > 0) yDir = -1;
            
            return new Vector2Int(xDir, yDir);
        }

        public CardinalDirection GetRelativeDirection(RelativeDirection dir) {
            return GetDirection(this.Index + (int) dir);
        }

        public static CardinalDirection[] AllDirections() => Directions;


    }
}