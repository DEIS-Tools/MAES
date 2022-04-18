using System;
using Maes.YamlConfig;

namespace Maes.Map.MapGen {
    public struct BuildingMapConfig {
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
        public readonly uint roomSplitChancePercent;

        public readonly int borderSize;

        public BuildingMapConfig(MaesYamlConfigLoader.MaesConfigType config, int seed) : this(
            widthInTiles: config.Map.WidthInTiles,
            heightInTiles: config.Map.HeightInTiles,
            randomSeed: seed,
            maxHallInPercent: config.Map.BuildingConfig.MaxHallInPercent,
            hallWidth: config.Map.BuildingConfig.HallWidth,
            minRoomSideLength: config.Map.BuildingConfig.MinRoomSideLength,
            doorWidth: config.Map.BuildingConfig.DoorWidth,
            doorPadding: config.Map.BuildingConfig.DoorPadding,
            roomSplitChancePercent: config.Map.BuildingConfig.RoomSplitChance,
            borderSize: config.Map.BorderSize
            ) {
            
        }
        
        public BuildingMapConfig(int widthInTiles, int heightInTiles, int randomSeed, float maxHallInPercent,
            int hallWidth, int minRoomSideLength, uint doorWidth, int doorPadding, uint roomSplitChancePercent,
            int borderSize) {
            if ((2 * doorPadding + doorWidth) > minRoomSideLength) {
                throw new ArgumentOutOfRangeException(
                    "Door width cannot be bigger than the smallest side lenght of rooms plus two times doorPadding");
            }

            if (roomSplitChancePercent > 100) {
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
            this.roomSplitChancePercent = roomSplitChancePercent;
            this.borderSize = borderSize;
        }
    }
}