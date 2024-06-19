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
using System.Collections.Generic;
using Maes.YamlConfig;

namespace Maes.Map.MapGen
{
    public struct CaveMapConfig
    {
        // CollisionMap size is always +1 larger in both axis
        // due to the marching squares algorithm using 4 points per square
        public int WidthInTiles { get; }
        public int HeightInTiles { get; }
        public int BitMapWidth { get; }
        public int BitMapHeight { get; }

        public int RandomSeed { get; }

        // How many runs of smoothing to get from QR code like noise to groups of room or wall tiles.
        public int SmoothingRuns { get; }

        // How wide should the passages made by the connection algorithm be
        public int ConnectionPassagesWidth { get; }

        // How much of the room should be filled with walls.
        public int RandomFillPercent { get; }

        // Minimum number of wall tiles in a group to not delete them in processing
        public int WallThresholdSize { get; }

        // Minimum number of rooms tiles in a group to not delete them in processing
        public int RoomThresholdSize { get; }

        // Border size (Assured walls on the edges)
        public int BorderSize { get; }

        public int NeighbourWallsNeededToStayWall { get; }


        internal CaveMapConfig(MaesYamlConfigLoader.MaesConfigType config, int seed) : this(
            randomSeed: seed,
            widthInTiles: config.Map.WidthInTiles,
            heightInTiles: config.Map.HeightInTiles,
            smoothingRuns: config.Map.CaveConfig.SmoothingRuns,
            connectionPassagesWidth: config.Map.CaveConfig.ConnectionPassageWidth,
            randomFillPercent: config.Map.CaveConfig.RandomFillPercent,
            wallThresholdSize: config.Map.CaveConfig.WallThresholdSize,
            roomThresholdSize: config.Map.CaveConfig.RoomThresholdSize,
            borderSize: config.Map.BorderSize)
        {
        }

        public CaveMapConfig(
            int randomSeed,
            int widthInTiles = 100, int heightInTiles = 100,
            int smoothingRuns = 3,
            int connectionPassagesWidth = 4,
            int randomFillPercent = 50,
            int wallThresholdSize = 1,
            int roomThresholdSize = 20,
            int borderSize = 2,
            int neighbourWallsNeededToStayWall = 3)
        {
            // Only fill percent between and including 0 to 100 are allowed
            if (0 > randomFillPercent || randomFillPercent >= 100)
            {
                throw new ArgumentOutOfRangeException("RandomFillPercent must be between 0 and 100");
            }

            if (smoothingRuns < 0)
            {
                throw new ArgumentOutOfRangeException("SmoothingRuns must be a positive integer or 0");
            }


            WidthInTiles = widthInTiles;
            HeightInTiles = heightInTiles;
            BitMapWidth = widthInTiles + 1 - (borderSize * 2);
            BitMapHeight = heightInTiles + 1 - (borderSize * 2);

            RandomSeed = randomSeed;
            SmoothingRuns = smoothingRuns;
            ConnectionPassagesWidth = connectionPassagesWidth;
            RandomFillPercent = randomFillPercent;
            WallThresholdSize = wallThresholdSize;
            RoomThresholdSize = roomThresholdSize;
            BorderSize = borderSize;

            NeighbourWallsNeededToStayWall = neighbourWallsNeededToStayWall;
        }
    }
}