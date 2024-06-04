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

using UnityEngine;

namespace Maes.Map.MapGen
{
    // Factory for generating maps using overloading
    public class MapSpawner: MonoBehaviour
    {
        public SimulationMap<Tile> GenerateMap(CaveMapConfig caveConfig, float wallHeight = 2.0f)
        {
            var caveGenerator = gameObject.AddComponent<CaveGenerator>();
            return caveGenerator.GenerateCaveMap(caveConfig,wallHeight);
        }
        
        public SimulationMap<Tile> GenerateMap(BuildingMapConfig buildingConfig, float wallHeight = 2.0f)
        {
            var buildingGenerator = gameObject.AddComponent<BuildingGenerator>(); 
            return buildingGenerator.GenerateBuildingMap(buildingConfig, wallHeight);
        }

        public SimulationMap<Tile> GenerateMap(Tile[,] bitmap, int seed, float wallHeight = 2.0f, int borderSize = 1)
        {
            var bitMapGenerator = gameObject.AddComponent<BitMapGenerator>();
            return bitMapGenerator.CreateMapFromBitMap(bitmap, seed, wallHeight, borderSize);
        }
    }
}