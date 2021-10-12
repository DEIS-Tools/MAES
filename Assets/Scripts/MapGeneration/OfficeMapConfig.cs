using System;

namespace Dora.MapGeneration
{
    public struct OfficeMapConfig
    {
        public int width;
        public int height;

        public string randomSeed;

        public float minSplittableArea;
        
        public float maxHallInPercent;

        public float hallWidth;
        public float minRoomSideLength;
        public float doorWidth;
        // Value in [0,100] determining the likelihood of an office being split
        // Higher value = more but smaller rooms.
        public uint officeSplitChance;

        public OfficeMapConfig(int width, int height, string randomSeed, float minSplittableArea, float maxHallInPercent, float hallWidth, float minRoomSideLength, float doorWidth, uint officeSplitChance) {
            if (doorWidth > minRoomSideLength) {
                throw new ArgumentOutOfRangeException("Door width cannot be bigger than the smallest side lenght of rooms");
            }

            if (officeSplitChance > 100) {
                throw new ArgumentOutOfRangeException("officeSplitChance cannot be greater than 100");
            }
            
            this.width = width;
            this.height = height;
            this.randomSeed = randomSeed;
            this.minSplittableArea = minSplittableArea;
            this.maxHallInPercent = maxHallInPercent;
            this.hallWidth = hallWidth;
            this.minRoomSideLength = minRoomSideLength;
            this.doorWidth = doorWidth;
            this.officeSplitChance = officeSplitChance;
        }
    }
}