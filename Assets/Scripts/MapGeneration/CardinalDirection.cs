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

        private static CardinalDirection[] _directions = new[]
            {East, SouthEast, South, SouthWest, West, NorthWest, North, NorthEast};
        
        public readonly int Index;
        
        // Can only be constructed locally. Must be accessed through public static instances
        private CardinalDirection(int index) {
            Index = index;
        }

        public CardinalDirection OppositeDirection() => GetDirection((Index + 4) % 8);
        public CardinalDirection DirectionToAngle() => GetDirection(((8 - Index) % 8) * 45); 
        public bool IsDirectionDiagonal() => Index % 2 != 0;

        public static CardinalDirection GetDirection(int index) {
            return _directions[index % 8];
        }
        
    }
}