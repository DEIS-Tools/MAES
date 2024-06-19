// Copyright 2024 MAES
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
// Contributors: Rasmus Borrisholt Schmidt, Andreas Sebastian Sørensen, Thor Beregaard
// 
// Original repository: https://github.com/Molitany/MAES

using System;
using System.Collections.Generic;

namespace Maes.Map.MapGen
{
    public enum TileType
    {
        Room,
        Hall,
        Wall,
        Concrete,
        Wood,
        Brick
    }

    public class Tile
    {

        public static Random Rand { get; set; }
        
        public TileType Type { get; }

        public Tile(TileType type)
        {
            Type = type;
        }

        public static Tile GetRandomWall()
        {
            if (Rand == null)
                throw new Exception("Random is not set");

            var typeValues = Enum.GetValues(typeof(TileType));
            var randomWallInt = Rand.Next((int)TileType.Concrete, typeValues.Length);
            return new Tile((TileType)typeValues.GetValue(randomWallInt));
        }

        public static bool IsWall(TileType tile)
        {
            return (int)tile >= (int)TileType.Wall;
        }

        public static TileType[] Walls()
        {
            var wallTypes = new List<TileType>();
            var typeValues = Enum.GetValues(typeof(TileType));
            for (var i = (int)TileType.Concrete; i < typeValues.Length; i++)
            {
                wallTypes.Add((TileType)typeValues.GetValue(i));
            }
            return wallTypes.ToArray();
        }
    }
}