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
// Contributors: Rasmus Borrisholt Schmidt, Andreas Sebastian Sørensen, Thor Beregaard, Malte Z. Andreasen, Philip I. Holler and Magnus K. Jensen,
// 
// Original repository: https://github.com/Molitany/MAES

using System.Collections.Generic;
using System.Linq;
using UnityEngine;


namespace Maes.Map.MapGen
{
    public abstract class MapGenerator : MonoBehaviour
    {
        protected Transform Plane;
        protected Transform InnerWalls2D;
        protected Transform InnerWalls3D;
        protected Transform WallRoof;
        private MeshGenerator _meshGenerator;

        // Variable used for drawing gizmos on selection for debugging.
        protected Tile[,] MapToDraw = null;

        public void Awake()
        {
            Plane = GameObject.Find("CaveFloor").GetComponent<Transform>();
            InnerWalls2D = GameObject.Find("InnerWalls2D").GetComponent<Transform>();
            InnerWalls3D = GameObject.Find("InnerWalls3D").GetComponent<Transform>();
            WallRoof = GameObject.Find("WallRoof").GetComponent<Transform>();
            _meshGenerator = GetComponent<MeshGenerator>();
        }

        protected void MovePlaneAndWallRoofToFitWallHeight(float wallHeight)
        {
            var newPosition = WallRoof.position;
            newPosition.z = -wallHeight;
            WallRoof.position = newPosition;

            newPosition = InnerWalls2D.position;
            newPosition.z = -wallHeight;
            InnerWalls2D.position = newPosition;
        }

        protected void ResizePlaneToFitMap(int bitMapHeight, int bitMapWidth, float padding = 0.1f)
        {
            // Resize plane below cave to fit size
            Plane.localScale = new Vector3(((bitMapWidth) / 10f) + padding,
                1,
                (bitMapHeight / 10f) + padding);
        }

        protected void ClearMap()
        {
            _meshGenerator.ClearMesh();
        }

        protected Tile[,] CreateBorderedMap(Tile[,] map, int width, int height, int borderSize)
        {
            var borderedMap = new Tile[width + (borderSize * 2), height + (borderSize * 2)];
            var tile = Tile.GetRandomWall();
            for (var x = 0; x < borderedMap.GetLength(0); x++)
            {
                for (var y = 0; y < borderedMap.GetLength(1); y++)
                {
                    if (x > borderSize - 1 && x < width + borderSize && y > borderSize - 1 && y < height + borderSize)
                    {
                        borderedMap[x, y] = map[x - borderSize, y - borderSize];
                    }
                    else
                    {
                        borderedMap[x, y] = tile;
                    }
                }
            }

            return borderedMap;
        }
        protected List<List<Vector2Int>> GetRegions(Tile[,] map, params TileType[] tileTypes)
        {
            var regions = new List<List<Vector2Int>>();
            // Flags if a given coordinate has already been accounted for
            // 1 = yes, 0 = no
            var mapFlags = new int[map.GetLength(0), map.GetLength(1)];
            const int counted = 1, notCounted = 0;

            for (var x = 0; x < map.GetLength(0); x++)
            {
                for (var y = 0; y < map.GetLength(1); y++)
                {
                    if (mapFlags[x, y] != notCounted || !tileTypes.Contains(map[x, y].Type))
                        continue;

                    var newRegion = GetRegionTiles(x, y, map);
                    regions.Add(newRegion);

                    foreach (var tile in newRegion)
                    {
                        mapFlags[tile.x, tile.y] = counted;
                    }
                }
            }

            return regions;
        }

        // A flood-full algorithm for finding all tiles in the region
        // For example if it starts at some point, that is an empty room tile
        // if will return all room tiles connected (in this region).
        // This is a similar algorithm to the one used in MS Paint for filling.
        protected List<Vector2Int> GetRegionTiles(int startX, int startY, Tile[,] map)
        {
            var tiles = new List<Vector2Int>();
            var mapFlags = new int[map.GetLength(0), map.GetLength(1)];
            const int counted = 1;
            const int notCounted = 0;
            var tileType = map[startX, startY].Type;

            var queue = new Queue<Vector2Int>();
            queue.Enqueue(new Vector2Int(startX, startY));
            mapFlags[startX, startY] = counted;

            while (queue.Count > 0)
            {
                var tile = queue.Dequeue();
                tiles.Add(tile);

                for (var x = tile.x - 1; x <= tile.x + 1; x++)
                {
                    for (var y = tile.y - 1; y <= tile.y + 1; y++)
                    {
                        if (!IsInMapRange(x, y, map) || (y != tile.y && x != tile.x))
                            continue;
                        if (mapFlags[x, y] != notCounted || map[x, y].Type != tileType)
                            continue;
                        mapFlags[x, y] = counted;
                        queue.Enqueue(new Vector2Int(x, y));
                    }
                }
            }

            return tiles;
        }
        protected bool IsInMapRange(int x, int y, Tile[,] map)
        {
            return x >= 0 && x < map.GetLength(0) && y >= 0 && y < map.GetLength(1);
        }

        protected (List<Room> surviningRooms, Tile[,] map) RemoveRoomsAndWallsBelowThreshold(int wallThreshold, int roomThreshold,
            Tile[,] map)
        {
            var cleanedMap = map.Clone() as Tile[,];
            var wallRegions = GetRegions(cleanedMap, Tile.Walls());

            foreach (var tile in wallRegions.Where(wallRegion => wallRegion.Count < wallThreshold).SelectMany(wallRegion => wallRegion))
            {
                cleanedMap![tile.x, tile.y] = new Tile(TileType.Room);
            }

            var roomRegions = GetRegions(cleanedMap, TileType.Room);
            var survivingRooms = new List<Room>();

            foreach (var roomRegion in roomRegions)
            {
                if (roomRegion.Count < roomThreshold)
                {
                    var tileType = Tile.GetRandomWall();
                    foreach (var tile in roomRegion)
                    {
                        cleanedMap![tile.x, tile.y] = tileType;
                    }
                }
                else
                {
                    survivingRooms.Add(new Room(roomRegion, cleanedMap));
                }
            }

            return (survivingRooms, cleanedMap);
        }

        // Draw the gizmo of the map for debugging purposes.
        protected void DrawMap(Tile[,] map)
        {
            if (MapToDraw == null)
                return;

            var width = map.GetLength(0);
            var height = map.GetLength(1);

            for (var x = 0; x < width; x++)
            {
                for (var y = 0; y < height; y++)
                {
                    Gizmos.color = map[x, y].Type switch
                    {
                        TileType.Wall => Color.black,
                        TileType.Room => Color.white,
                        TileType.Hall => Color.gray,
                        TileType.Concrete => Color.yellow,
                        TileType.Wood => Color.green,
                        TileType.Brick => Color.red,
                        _ => Color.blue
                    };

                    var pos = new Vector3(-width / 2 + x + .5f, -height / 2 + y + .5f, 0);
                    Gizmos.DrawCube(pos, Vector3.one);
                }
            }
        }

        private void OnDrawGizmosSelected()
        {
            DrawMap(MapToDraw);
        }
    }
}