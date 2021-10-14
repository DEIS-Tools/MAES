using System;

namespace Dora.MapGeneration
{
    public struct OfficeMapConfig
    {
        public int width;
        public int height;

        public string randomSeed;

        public float maxHallInPercent;

        public int hallWidth;
        public float minRoomSideLength;
        public uint doorWidth;
        public int doorPadding;
        // Value in [0,100] determining the likelihood of an office being split
        // Higher value = more but smaller rooms.
        public uint officeSplitChancePercent;

        public int borderSize;

        public float scaling;

        public OfficeMapConfig(int width, int height, string randomSeed, float maxHallInPercent, int hallWidth, float minRoomSideLength, uint doorWidth, int doorPadding, uint officeSplitChancePercent, int borderSize, float scaling) {
            if ((2 * doorPadding  + doorWidth) > minRoomSideLength) {
                throw new ArgumentOutOfRangeException("Door width cannot be bigger than the smallest side lenght of rooms plus two times doorPadding");
            }

            if (officeSplitChancePercent > 100) {
                throw new ArgumentOutOfRangeException("officeSplitChance cannot be greater than 100");
            }
            
            this.width = width;
            this.height = height;
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