using UnityEngine;

namespace Maes.Map.MapGen
{
    // Factory for generating maps using overloading
    public class MapSpawner: MonoBehaviour
    {
        public SimulationMap<Tile> GenerateMap(CaveMapConfig caveConfig, float wallHeight = 2.0f)
        {
            var caveGenerator = gameObject.AddComponent<CaveGenerator>();
            gameObject.SetActive(false);
            return caveGenerator.GenerateCaveMap(caveConfig,wallHeight);
        }
        
        public SimulationMap<Tile> GenerateMap(BuildingMapConfig buildingConfig, float wallHeight = 2.0f)
        {
            var buildingGenerator = gameObject.AddComponent<BuildingGenerator>(); 
            gameObject.SetActive(false);
            return buildingGenerator.GenerateBuildingMap(buildingConfig, wallHeight);
        }

        public SimulationMap<Tile> GenerateMap(Tile[,] bitmap, float wallHeight = 2.0f, int borderSize = 1)
        {
            var bitMapGenerator = gameObject.AddComponent<BitMapGenerator>();
            gameObject.SetActive(false);
            return bitMapGenerator.CreateMapFromBitMap(bitmap, wallHeight, borderSize);
        }
    }
}