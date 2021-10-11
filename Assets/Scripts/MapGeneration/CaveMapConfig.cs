using System;

namespace Dora.MapGeneration
{
    public struct CaveMapConfig
    {
        public int width;
        public int height;

        public string randomSeed;
	
        // How many runs of smoothing to get from QR code like noise to groups of room or wall tiles.
        public int smoothingRuns;
	
        // How wide should the passages made by the connection algorithm be
        public int connectionPassagesWidth;

        // How much of the room should be filled with walls.
        public int randomFillPercent;
	
        // Minimum number of wall tiles in a group to not delete them in processing
        public int wallThresholdSize;
	
        // Minimum number of rooms tiles in a group to not delete them in processing
        public int roomThresholdSize;
	
        // Border size (Assured walls on the edges)
        public int borderSize;

        // This can be increased to enlarge the smallest corridors by enlarging the entire cave
        public int scaling;

        public int neighbourWallsNeededToStayWall;

        public CaveMapConfig(int width, int height, string randomSeed, int smoothingRuns, int connectionPassagesWidth, int randomFillPercent, int wallThresholdSize, int roomThresholdSize, int borderSize, int scaling, int neighbourWallsNeededToStayWall = 4)
        {
	        // Only fill percent between and including 0 to 100 are allowed
	        if(0 >= randomFillPercent || randomFillPercent >= 100 ){
		        throw new ArgumentOutOfRangeException("randomFillPercent must be between 0 and 100");
	        }

	        if (smoothingRuns < 0)
	        {
		        throw new ArgumentOutOfRangeException("smoothingRuns must be a positive integer or 0");
	        }
	        
	        
	        this.width = width;
	        this.height = height;
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