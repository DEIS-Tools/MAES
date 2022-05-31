// Copyright 2022 MAES
// 
// This file is part of MAES
// 
// MAES is free software: you can redistribute it and/or modify it under
// the terms of the GNU General Public License as published by the
// Free Software Foundation, either version 3 of the License, or (at your option)
// any later version.
// 
// MAES is distributed in the hope that it will be useful, but WITHOUT
// ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
// or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
// Public License for more details.
// 
// You should have received a copy of the GNU General Public License along
// with MAES. If not, see http://www.gnu.org/licenses/.
// 
// Contributors: Malte Z. Andreasen, Philip I. Holler and Magnus K. Jensen
// 
// Original repository: https://github.com/MalteZA/MAES

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
            randomSeed: seed,
            widthInTiles: config.Map.WidthInTiles,
            heightInTiles: config.Map.HeightInTiles,
            maxHallInPercent: config.Map.BuildingConfig.MaxHallInPercent,
            hallWidth: config.Map.BuildingConfig.HallWidth,
            minRoomSideLength: config.Map.BuildingConfig.MinRoomSideLength,
            doorWidth: config.Map.BuildingConfig.DoorWidth,
            doorPadding: config.Map.BuildingConfig.DoorPadding,
            roomSplitChancePercent: config.Map.BuildingConfig.RoomSplitChance,
            borderSize: config.Map.BorderSize
            ) {
            
        }
        
        public BuildingMapConfig(
            int randomSeed, 
            int widthInTiles=50, int heightInTiles=50, 
            float maxHallInPercent=20,
            int hallWidth=4, 
            int minRoomSideLength=6, 
            uint doorWidth=2, 
            int doorPadding=2, 
            uint roomSplitChancePercent=85,
            int borderSize=1) {
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