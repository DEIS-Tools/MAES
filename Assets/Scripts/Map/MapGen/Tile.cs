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

namespace Maes.Map.MapGen
{
    public enum TileType
    {
        Room,
        Hall,
        Wall,
        Concrete,
        Wood,
        Metal
    }

    public class Tile
    {

        public static Random Rand { get; set; }
        //todo add attenuation for air and water
        private static readonly Dictionary<TileType, int> _attenuationDictionary = new()
        {
            [TileType.Room] = 0,
            [TileType.Hall] = 0,
            [TileType.Wall] = 0,
            [TileType.Concrete] = 8, // 8-15 2.4GHz/1.3GHz
            [TileType.Wood] = 3, // 2.4GHz
            [TileType.Metal] = 26 // 815MHz
        };

        public TileType Type { get; }
        public int Attenuation { get; }

        public Tile(TileType type)
        {
            Type = type;
            Attenuation = _attenuationDictionary[type];
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
            return (int)tile >= (int)TileType.Concrete;
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