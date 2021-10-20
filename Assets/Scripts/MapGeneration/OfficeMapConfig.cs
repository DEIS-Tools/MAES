using System;

namespace Dora.MapGeneration
{
    public struct OfficeMapConfig
    {
        // Bitmap size is always +1 larger in both axis
        // due to the marching squares algorithm using 4 points per square
        public readonly int widthInTiles;
        public readonly int heightInTiles;
        public readonly int bitMapWidth;
        public readonly int bitMapHeight;

        public readonly int randomSeed;

        public readonly float maxHallInPercent;

        public readonly int hallWidth;
        public readonly float minRoomSideLength;
        public readonly uint doorWidth;
        public readonly int doorPadding;
        // Value in [0,100] determining the likelihood of an office being split
        // Higher value = more but smaller rooms.
        public readonly uint officeSplitChancePercent;

        public readonly int borderSize;

        public readonly float scaling;

        public OfficeMapConfig(int widthInTiles, int heightInTiles, int randomSeed, float maxHallInPercent, int hallWidth, float minRoomSideLength, uint doorWidth, int doorPadding, uint officeSplitChancePercent, int borderSize, float scaling) {
            if ((2 * doorPadding  + doorWidth) > minRoomSideLength) {
                throw new ArgumentOutOfRangeException("Door width cannot be bigger than the smallest side lenght of rooms plus two times doorPadding");
            }

            if (officeSplitChancePercent > 100) {
                throw new ArgumentOutOfRangeException("officeSplitChance cannot be greater than 100");
            }
            
            this.widthInTiles = widthInTiles;
            this.heightInTiles = heightInTiles;
            this.bitMapWidth = widthInTiles + 1 - (borderSize * 2);
            this.bitMapHeight = heightInTiles + 1 - (borderSize * 2);

            this.randomSeed = randomSeed;
            this.maxHallInPercent = maxHallInPercent;
            this.hallWidth = hallWidth;
            this.minRoomSideLength = minRoomSideLength;
            this.doorWidth = doorWidth;
            this.doorPadding = doorPadding;
            this.officeSplitChancePercent = officeSplitChancePercent;
            this.borderSize = borderSize;
            this.scaling = scaling;
        }
    }
    
    
}