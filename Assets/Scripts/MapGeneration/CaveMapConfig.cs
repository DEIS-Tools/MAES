using System;

namespace Dora.MapGeneration {
    public struct CaveMapConfig {
        // CollisionMap size is always +1 larger in both axis
        // due to the marching squares algorithm using 4 points per square
        public readonly int widthInTiles;
        public readonly int heightInTiles;
        public readonly int bitMapWidth;
        public readonly int bitMapHeight;

        public readonly int randomSeed;

        // How many runs of smoothing to get from QR code like noise to groups of room or wall tiles.
        public readonly int smoothingRuns;

        // How wide should the passages made by the connection algorithm be
        public readonly int connectionPassagesWidth;

        // How much of the room should be filled with walls.
        public readonly int randomFillPercent;

        // Minimum number of wall tiles in a group to not delete them in processing
        public readonly int wallThresholdSize;

        // Minimum number of rooms tiles in a group to not delete them in processing
        public readonly int roomThresholdSize;

        // Border size (Assured walls on the edges)
        public readonly int borderSize;

        // This can be increased to enlarge the smallest corridors by enlarging the entire cave
        public readonly float scaling;

        public readonly int neighbourWallsNeededToStayWall;

        public CaveMapConfig(int widthInTiles, int heightInTiles, int randomSeed, int smoothingRuns,
            int connectionPassagesWidth, int randomFillPercent, int wallThresholdSize, int roomThresholdSize,
            int borderSize, float scaling, int neighbourWallsNeededToStayWall = 4) {
            // Only fill percent between and including 0 to 100 are allowed
            if (0 > randomFillPercent || randomFillPercent >= 100) {
                throw new ArgumentOutOfRangeException("randomFillPercent must be between 0 and 100");
            }

            if (smoothingRuns < 0) {
                throw new ArgumentOutOfRangeException("smoothingRuns must be a positive integer or 0");
            }


            this.widthInTiles = widthInTiles;
            this.heightInTiles = heightInTiles;
            this.bitMapWidth = widthInTiles + 1 - (borderSize * 2);
            this.bitMapHeight = heightInTiles + 1 - (borderSize * 2);

            this.randomSeed = randomSeed;
            this.smoothingRuns = smoothingRuns;
            this.connectionPassagesWidth = connectionPassagesWidth;
            this.randomFillPercent = randomFillPercent;
            this.wallThresholdSize = wallThresholdSize;
            this.roomThresholdSize = roomThresholdSize;
            this.borderSize = borderSize;
            this.scaling = scaling;
            this.neighbourWallsNeededToStayWall = neighbourWallsNeededToStayWall;
        }
    }
}