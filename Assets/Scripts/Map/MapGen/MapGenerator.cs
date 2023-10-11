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

using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using static Maes.Map.MapGen.BitMapTypes;


namespace Maes.Map.MapGen
{
    //TODO: https://stackoverflow.com/questions/49182521/unity3d-add-component-to-prefab-in-resource
    public abstract class MapGenerator : MonoBehaviour
    {
        protected Transform _plane;
        protected Transform _innerWalls2D;
        protected Transform _innerWalls3D;
        protected Transform _wallRoof;
        private MeshGenerator _meshGenerator;

        // Variable used for drawing gizmos on selection for debugging.
        protected int[,] mapToDraw = null;

        public void Awake()
        {
            _plane = GameObject.Find("CaveFloor").GetComponent<Transform>();
            _innerWalls2D = GameObject.Find("InnerWalls2D").GetComponent<Transform>();
            _innerWalls3D = GameObject.Find("InnerWalls3D").GetComponent<Transform>();
            _wallRoof = GameObject.Find("WallRoof").GetComponent<Transform>();
            _meshGenerator = GetComponent<MeshGenerator>();
        }

        protected void MovePlaneAndWallRoofToFitWallHeight(float wallHeight)
        {
            Vector3 newPosition = _wallRoof.position;
            newPosition.z = -wallHeight;
            _wallRoof.position = newPosition;

            newPosition = _innerWalls2D.position;
            newPosition.z = -wallHeight;
            _innerWalls2D.position = newPosition;
        }

        protected void ResizePlaneToFitMap(int bitMapHeight, int bitMapWidth, float padding = 0.1f)
        {
            // Resize plane below cave to fit size
            _plane.localScale = new Vector3(((bitMapWidth) / 10f) + padding,
                1,
                (bitMapHeight / 10f) + padding);
        }

        protected void clearMap()
        {
            _meshGenerator.ClearMesh();
        }

        protected int[,] CreateBorderedMap(int[,] map, int width, int height, int borderSize)
        {
            int[,] borderedMap = new int[width + (borderSize * 2), height + (borderSize * 2)];

            for (int x = 0; x < borderedMap.GetLength(0); x++)
            {
                for (int y = 0; y < borderedMap.GetLength(1); y++)
                {
                    if (x >= borderSize && x < width + borderSize && y >= borderSize && y < height + borderSize)
                    {
                        borderedMap[x, y] = map[x - borderSize, y - borderSize];
                    }
                    else
                    {
                        borderedMap[x, y] = WALL_TYPE;
                    }
                }
            }

            return borderedMap;
        }
        protected List<List<Vector2Int>> GetRegions(int[,] map, params int[] tileTypes)
        {
            List<List<Vector2Int>> regions = new List<List<Vector2Int>>();
            // Flags if a given coordinate has already been accounted for
            // 1 = yes, 0 = no
            int[,] mapFlags = new int[map.GetLength(0), map.GetLength(1)];
            int counted = 1, notCounted = 0;

            for (int x = 0; x < map.GetLength(0); x++)
            {
                for (int y = 0; y < map.GetLength(1); y++)
                {
                    if (mapFlags[x, y] == notCounted && tileTypes.Contains(map[x, y]))
                    {
                        List<Vector2Int> newRegion = GetRegionTiles(x, y, map);
                        regions.Add(newRegion);

                        foreach (Vector2Int tile in newRegion)
                        {
                            mapFlags[tile.x, tile.y] = counted;
                        }
                    }
                }
            }

            return regions;
        }

        // A flood-full algorithm for finding all tiles in the region
        // For example if it starts at some point, that is an empty room tile
        // if will return all room tiles connected (in this region).
        // This is a similar algorithm to the one used in MS Paint for filling.
        protected List<Vector2Int> GetRegionTiles(int startX, int startY, int[,] map)
        {
            List<Vector2Int> tiles = new List<Vector2Int>();
            int[,] mapFlags = new int[map.GetLength(0), map.GetLength(1)];
            int counted = 1, notCounted = 0;
            int tileType = map[startX, startY];

            Queue<Vector2Int> queue = new Queue<Vector2Int>();
            queue.Enqueue(new Vector2Int(startX, startY));
            mapFlags[startX, startY] = counted;

            while (queue.Count > 0)
            {
                Vector2Int tile = queue.Dequeue();
                tiles.Add(tile);

                for (int x = tile.x - 1; x <= tile.x + 1; x++)
                {
                    for (int y = tile.y - 1; y <= tile.y + 1; y++)
                    {
                        if (IsInMapRange(x, y, map) && (y == tile.y || x == tile.x))
                        {
                            if (mapFlags[x, y] == notCounted && map[x, y] == tileType)
                            {
                                mapFlags[x, y] = counted;
                                queue.Enqueue(new Vector2Int(x, y));
                            }
                        }
                    }
                }
            }

            return tiles;
        }
        protected bool IsInMapRange(int x, int y, int[,] map)
        {
            return x >= 0 && x < map.GetLength(0) && y >= 0 && y < map.GetLength(1);
        }

        protected (List<Room> surviningRooms, int[,] map) RemoveRoomsAndWallsBelowThreshold(int wallThreshold, int roomThreshold,
            int[,] map)
        {
            var cleanedMap = map.Clone() as int[,];
            List<List<Vector2Int>> wallRegions = GetRegions(cleanedMap, WALL_TYPE);

            foreach (List<Vector2Int> wallRegion in wallRegions)
            {
                if (wallRegion.Count < wallThreshold)
                {
                    foreach (Vector2Int tile in wallRegion)
                    {
                        cleanedMap[tile.x, tile.y] = ROOM_TYPE;
                    }
                }
            }

            List<List<Vector2Int>> roomRegions = GetRegions(cleanedMap, ROOM_TYPE);
            List<Room> survivingRooms = new List<Room>();

            foreach (List<Vector2Int> roomRegion in roomRegions)
            {
                if (roomRegion.Count < roomThreshold)
                {
                    foreach (Vector2Int tile in roomRegion)
                    {
                        cleanedMap[tile.x, tile.y] = WALL_TYPE;
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
        private void drawMap(int[,] map)
        {
            if (mapToDraw != null)
            {
                int width = map.GetLength(0);
                int height = map.GetLength(1);

                for (int x = 0; x < width; x++)
                {
                    for (int y = 0; y < height; y++)
                    {
                        switch (map[x, y])
                        {
                            case WALL_TYPE:
                                Gizmos.color = Color.black;
                                break;
                            case ROOM_TYPE:
                                Gizmos.color = Color.white;
                                break;
                            case HALL_TYPE:
                                Gizmos.color = Color.gray;
                                break;
                            default:
                                Gizmos.color = Color.red;
                                break;
                        }

                        Vector3 pos = new Vector3(-width / 2 + x + .5f, 0, -height / 2 + y + .5f);
                        Gizmos.DrawCube(pos, Vector3.one);
                    }
                }
            }
        }

        private void OnDrawGizmosSelected()
        {
            drawMap(mapToDraw);
        }
    }
}