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
        public float doorWidth;


        public OfficeMapConfig(int width, int height, string randomSeed, float minSplittableArea, float maxHallInPercent, float hallWidth, float doorWidth) {
            this.width = width;
            this.height = height;
            this.randomSeed = randomSeed;
            this.minSplittableArea = minSplittableArea;
            this.maxHallInPercent = maxHallInPercent;
            this.hallWidth = hallWidth;
            this.doorWidth = doorWidth;
        }
    }
}