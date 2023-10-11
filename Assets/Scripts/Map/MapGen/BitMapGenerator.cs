using System;
using UnityEngine;

namespace Maes.Map.MapGen
{
    public class BitMapGenerator : MapGenerator
    {
        private int[,] _bitmap;
        private float _wallHeight;
        private int _borderSize;

        public void Init(int[,] bitmap, float wallHeight = 2.0f, int borderSize = 1)
        {
            _bitmap = bitmap;
            _wallHeight = wallHeight;
            _borderSize = borderSize;
        }

        /// <summary>
        /// Method for creating a map from an array of ints {0, 1}.
        /// </summary>
        public SimulationMap<bool> CreateMapFromBitMap()
        {
            // Clear and destroy objects from previous map
            clearMap();

            // TODO: If the border size is less than two, sometimes the mesh is generated with wierd invisible walls
            // Can be reproduced by having a map with a line with a width of 2 going through the middle and splitting the map
            // and having a border size smaller than 2. The collider also covers the outside of the map, when the bug happens.
            _borderSize = Math.Max(2, _borderSize);

            // Add border around map
            var borderedMap = CreateBorderedMap(_bitmap, _bitmap.GetLength(0), _bitmap.GetLength(1), _borderSize);

            // Get rooms needed for mesh creation
            var (survivingRooms, cleanedMap) = RemoveRoomsAndWallsBelowThreshold(0, 0, borderedMap);

            // The rooms should now reflect their relative shifted positions after adding borders round map.
            survivingRooms.ForEach(r => r.OffsetCoordsBy(_borderSize, _borderSize));

            // Create mesh
            MeshGenerator meshGen = GetComponent<MeshGenerator>();
            var collisionMap = meshGen.GenerateMesh(cleanedMap.Clone() as int[,], _wallHeight,
                true, survivingRooms);

            // Rotate to fit 2D view
            _plane.rotation = Quaternion.AngleAxis(-90, Vector3.right);
            ResizePlaneToFitMap(_bitmap.GetLength(1), _bitmap.GetLength(0));
            MovePlaneAndWallRoofToFitWallHeight(_wallHeight);

            return collisionMap;
        }

    }
}