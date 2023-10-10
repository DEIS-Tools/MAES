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

using UnityEngine;


namespace Maes.Map.MapGen
{
    public class MapSpawner: MonoBehaviour
    {
        public SimulationMap<bool> GenerateMap(CaveMapConfig caveConfig, float wallHeight = 2.0f)
        {
            var caveGenerator = gameObject.AddComponent<CaveGenerator>();
            caveGenerator.Init(caveConfig, wallHeight);
            var result = caveGenerator.GenerateCaveMap();
            Destroy(caveGenerator);
            return result;
        }
        
        public SimulationMap<bool> GenerateMap(BuildingMapConfig buildingConfig, float wallHeight = 2.0f)
        {
            var buildingGenerator = gameObject.AddComponent<BuildingGenerator>(); 
            buildingGenerator.Init(buildingConfig, wallHeight);
            var result = buildingGenerator.GenerateBuildingMap();
            Destroy(buildingGenerator);
            return result;
        }

        public SimulationMap<bool> GenerateMap(int[,] bitmap, float wallHeight = 2.0f, int borderSize = 1)
        {
            var bitMapGenerator = gameObject.AddComponent<BitMapGenerator>();
            bitMapGenerator.Init(bitmap, wallHeight, borderSize);
            var result = bitMapGenerator.CreateMapFromBitMap();
            Destroy(bitMapGenerator);
            return result;
        }
    }
}