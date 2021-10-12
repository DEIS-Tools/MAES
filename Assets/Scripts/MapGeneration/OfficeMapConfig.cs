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


        public OfficeMapConfig(int width, int height, string randomSeed, float minSplittableArea, float maxHallInPercent, float hallWidth, float minRoomSideLength, float doorWidth) {
            if (doorWidth < minRoomSideLength) {
                throw new ArgumentOutOfRangeException("Door width cannot be bigger than the smallest side lenght of rooms");
            }

            this.width = width;
            this.height = height;
            this.randomSeed = randomSeed;
            this.minSplittableArea = minSplittableArea;
            this.maxHallInPercent = maxHallInPercent;
            this.hallWidth = hallWidth;
            this.minRoomSideLength = minRoomSideLength;
            this.doorWidth = doorWidth;
        }
    }
}