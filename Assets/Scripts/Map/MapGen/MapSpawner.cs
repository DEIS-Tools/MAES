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