using System;
using UnityEngine;

namespace Maes.Map.MapGen
{
    public class BitMapGenerator : MapGenerator
    {
        private Tile[,] _bitmap;
        private float _wallHeight;
        private int _borderSize;

        /// <summary>
        /// Method for creating a map from a 2D array of Tiles.
        /// </summary>
        public SimulationMap<Tile> CreateMapFromBitMap(Tile[,] bitmap, int seed, float wallHeight = 2.0f, int borderSize = 1)
        {
            _bitmap = bitmap;
            _wallHeight = wallHeight;
            _borderSize = borderSize;
            Tile.Rand = new System.Random(seed);

            _borderSize = Math.Max(2, borderSize);

            // Clear and destroy objects from previous map
            ClearMap();

            // Add border around map
            var borderedMap = CreateBorderedMap(_bitmap, _bitmap.GetLength(0), _bitmap.GetLength(1), _borderSize);

            // Get rooms needed for mesh creation
            var (survivingRooms, cleanedMap) = RemoveRoomsAndWallsBelowThreshold(0, 0, borderedMap);

            // The rooms should now reflect their relative shifted positions after adding borders round map.
            survivingRooms.ForEach(r => r.OffsetCoordsBy(_borderSize, _borderSize));

            MapToDraw = cleanedMap;

            // Create mesh
            MeshGenerator meshGen = GetComponent<MeshGenerator>();
            var collisionMap = meshGen.GenerateMesh(cleanedMap.Clone() as Tile[,], _wallHeight,
                true, survivingRooms);

            // Rotate to fit 2D view
            Plane.rotation = Quaternion.AngleAxis(-90, Vector3.right);
            ResizePlaneToFitMap(_bitmap.GetLength(1), _bitmap.GetLength(0));
            MovePlaneAndWallRoofToFitWallHeight(_wallHeight);

            return collisionMap;
        }

        private void OnDrawGizmosSelected()
        {
            DrawMap(MapToDraw);
        }
    }
}
