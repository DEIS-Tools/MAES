using System;
using System.Collections.Generic;
using UnityEngine;
using static Maes.Map.MapGen.TileTypes;
using Quaternion = UnityEngine.Quaternion;
using Vector3 = UnityEngine.Vector3;

namespace Maes.Map.MapGen
{
    public class CaveGenerator : MapGenerator
    {
        private CaveMapConfig _caveConfig;
        private float _wallHeight;

        /// <summary>
        /// Generates a cave map using the unity game objects Plane, InnerWalls and WallRoof.
        /// </summary>
        /// <param name="caveConfig">Determines how the cave map should look</param>
        /// <param name="wallHeight">A smaller wall height can make it easier to see the robots. Must be a positive value.</param>
        /// <returns> A SimulationMap represents a map of square tiles, where each tile is divided into 8 triangles as
        /// used in the Marching Squares Algorithm.</returns>
        public void Init(CaveMapConfig config, float wallHeight = 2.0f)
        {
            _caveConfig = config;
            _wallHeight = wallHeight;
        }

        public SimulationMap<bool> GenerateCaveMap()
        {
            // Clear and destroy objects from previous map
            clearMap();

            var collisionMap = CreateCaveMapWithMesh(_caveConfig, _wallHeight);

            ResizePlaneToFitMap(_caveConfig.bitMapHeight, _caveConfig.bitMapWidth);

            MovePlaneAndWallRoofToFitWallHeight(_wallHeight);

            return collisionMap;
        }

        private SimulationMap<bool> CreateCaveMapWithMesh(CaveMapConfig caveConfig, float wallHeight = 2.0f)
        {
            // Fill map with random walls and empty tiles (Looks kinda like a QR code)
            var randomlyFilledMap = CreateRandomFillMap(caveConfig);

            // Use smoothing runs to make sense of the noise
            // f.x. walls can only stay walls, if they have at least N neighbouring walls
            int[,] smoothedMap = randomlyFilledMap;
            for (int i = 0; i < caveConfig.smoothingRuns; i++)
            {
                smoothedMap = SmoothMap(smoothedMap, caveConfig);
            }
                
            var smoothedMapWithoutNarrowCorridors = WallOffNarrowCorridors(smoothedMap);

            // Clean up regions smaller than threshold for both walls and rooms.
            var (survivingRooms, cleanedMap) = RemoveRoomsAndWallsBelowThreshold(caveConfig.wallThresholdSize,
                caveConfig.roomThresholdSize,
                smoothedMapWithoutNarrowCorridors);

            // Connect all rooms to main (the biggest) room
            var connectedMap = ConnectAllRoomsToMainRoom(survivingRooms, cleanedMap, caveConfig);

            // Ensure a border around the map
            var borderedMap = CreateBorderedMap(connectedMap, caveConfig.bitMapWidth, caveConfig.bitMapHeight,
                caveConfig.borderSize);

            // Draw gizmo of map for debugging. Will draw the map in Scene upon selection.
            // mapToDraw = borderedMap;

            // The rooms should now reflect their relative shifted positions after adding borders round map.
            survivingRooms.ForEach(r => r.OffsetCoordsBy(caveConfig.borderSize, caveConfig.borderSize));

            MeshGenerator meshGen = GetComponent<MeshGenerator>();
            var collisionMap = meshGen.GenerateMesh(borderedMap.Clone() as int[,], wallHeight,
                false, survivingRooms);

            // Rotate to fit 2D view
            _plane.rotation = Quaternion.AngleAxis(-90, Vector3.right);

            return collisionMap;
        }

        /**
     * In order to make sure, that all algorithms can traverse all tiles the following must hold:
     * - Every tile has 1 horizontal neighbour to each side, or 2 in either direction
     * - Every tile has 1 vertical neighbour to each side, or 2 in either direction
     * - Every tile has at least one diagonal neighbor on the line from bottom left to top right and top left to bottom right.
     * This is due to some algorithms (e.g. B&M) assuming that any partially covered tile is completely covered
     * This assumption leads to some narrow corridors traversable.
     * If we block the corridors with walls in this function, the algorithm will consider them two separate rooms later on
     * which will cause the algorithm to create a corridor of width n between them. This ensures traversability of all tiles. 
     */
        int[,] WallOffNarrowCorridors(int[,] map)
        {
            var newMap = map.Clone() as int[,];
            Queue<(int, int)> tilesToCheck = new Queue<(int, int)>();

            // Populate queue with all tiles
            for (int x = 0; x < newMap.GetLength(0); x++)
            {
                for (int y = 0; y < newMap.GetLength(1); y++)
                {
                    if (newMap[x, y] == ROOM_TYPE)
                        tilesToCheck.Enqueue((x, y));
                }
            }

            while (tilesToCheck.Count > 0)
            {
                var tile = tilesToCheck.Dequeue();
                var x = tile.Item1;
                var y = tile.Item2;
                if (newMap[x, y] == WALL_TYPE)
                    continue;

                bool makeSolid = false;
                // Check 3 tiles horizontally
                bool horisontalClear = false;
                if ((IsInMapRange(x - 1, y, newMap) && newMap[x - 1, y] == ROOM_TYPE) &&
                    (IsInMapRange(x + 1, y, newMap) && newMap[x + 1, y] == ROOM_TYPE))
                    horisontalClear = true;
                if ((IsInMapRange(x + 1, y, newMap) && newMap[x + 1, y] == ROOM_TYPE) &&
                    (IsInMapRange(x + 2, y, newMap) && newMap[x + 2, y] == ROOM_TYPE))
                    horisontalClear = true;
                if ((IsInMapRange(x - 1, y, newMap) && newMap[x - 1, y] == ROOM_TYPE) &&
                    (IsInMapRange(x - 2, y, newMap) && newMap[x - 2, y] == ROOM_TYPE))
                    horisontalClear = true;

                // Check 3 tiles vertically
                bool verticalClear = false;
                if ((IsInMapRange(x, y - 1, newMap) && newMap[x, y - 1] == ROOM_TYPE) &&
                    (IsInMapRange(x, y + 1, newMap) && newMap[x, y + 1] == ROOM_TYPE))
                    verticalClear = true;
                if ((IsInMapRange(x, y + 1, newMap) && newMap[x, y + 1] == ROOM_TYPE) &&
                    (IsInMapRange(x, y + 2, newMap) && newMap[x, y + 2] == ROOM_TYPE))
                    verticalClear = true;
                if ((IsInMapRange(x, y - 1, newMap) && newMap[x, y - 1] == ROOM_TYPE) &&
                    (IsInMapRange(x, y - 2, newMap) && newMap[x, y - 2] == ROOM_TYPE))
                    verticalClear = true;

                // Check 2 tiles from bottom left to top right clear
                bool bottomLeftToTopRightClear = false;
                if ((IsInMapRange(x - 1, y - 1, newMap) && newMap[x - 1, y - 1] == ROOM_TYPE))
                    bottomLeftToTopRightClear = true;
                if ((IsInMapRange(x + 1, y + 1, newMap) && newMap[x + 1, y + 1] == ROOM_TYPE))
                    bottomLeftToTopRightClear = true;

                // Check 2 tiles from top left to bottom right clear
                bool topLeftToBottomRightClear = false;
                if ((IsInMapRange(x - 1, y + 1, newMap) && newMap[x - 1, y + 1] == ROOM_TYPE))
                    topLeftToBottomRightClear = true;
                if ((IsInMapRange(x + 1, y - 1, newMap) && newMap[x + 1, y - 1] == ROOM_TYPE))
                    topLeftToBottomRightClear = true;

                if (!(horisontalClear && verticalClear && bottomLeftToTopRightClear && topLeftToBottomRightClear))
                {
                    newMap[x, y] = WALL_TYPE;
                    // enqueue neighbours to be checked again
                    for (int neighborX = x-1; neighborX < x+1; neighborX++)
                        for (int neighborY = y-1; neighborY < y+1; neighborY++)
                            if (IsInMapRange(neighborX, neighborY, newMap))
                                tilesToCheck.Enqueue((neighborX,neighborY));
                }
            }
            
            return newMap;
        }

        private int[,] ConnectAllRoomsToMainRoom(List<Room> survivingRooms, int[,] map, CaveMapConfig config)
        {
            var connectedMap = map.Clone() as int[,];
            survivingRooms.Sort();
            survivingRooms[0].isMainRoom = true;
            survivingRooms[0].isAccessibleFromMainRoom = true;

            return ConnectClosestRooms(survivingRooms, connectedMap, config.connectionPassagesWidth);
        }

        private int[,] ConnectClosestRooms(List<Room> allRooms, int[,] map, int passageWidth)
        {
            int[,] connectedMap = map.Clone() as int[,];
            List<Room> roomListA = new List<Room>();
            List<Room> roomListB = new List<Room>();


            foreach (Room room in allRooms)
            {
                if (room.isAccessibleFromMainRoom)
                {
                    roomListB.Add(room);
                }
                else
                {
                    roomListA.Add(room);
                }
            }


            int bestDistance = 0;
            Vector2Int bestTileA = new Vector2Int();
            Vector2Int bestTileB = new Vector2Int();
            Room bestRoomA = new Room();
            Room bestRoomB = new Room();
            bool possibleConnectionFound = false;

            foreach (Room roomA in roomListA)
            {
                foreach (Room roomB in roomListB)
                {
                    if (roomA == roomB || roomA.IsConnected(roomB))
                    {
                        continue;
                    }

                    for (int tileIndexA = 0; tileIndexA < roomA.edgeTiles.Count; tileIndexA++)
                    {
                        for (int tileIndexB = 0; tileIndexB < roomB.edgeTiles.Count; tileIndexB++)
                        {
                            Vector2Int tileA = roomA.edgeTiles[tileIndexA];
                            Vector2Int tileB = roomB.edgeTiles[tileIndexB];
                            int distanceBetweenRooms =
                                (int)(Mathf.Pow(tileA.x - tileB.x, 2) + Mathf.Pow(tileA.y - tileB.y, 2));

                            if (distanceBetweenRooms < bestDistance || !possibleConnectionFound)
                            {
                                bestDistance = distanceBetweenRooms;
                                possibleConnectionFound = true;
                                bestTileA = tileA;
                                bestTileB = tileB;
                                bestRoomA = roomA;
                                bestRoomB = roomB;
                            }
                        }
                    }
                }
            }

            if (possibleConnectionFound)
            {
                CreatePassage(bestRoomA, bestRoomB, bestTileA, bestTileB, connectedMap, passageWidth);
                connectedMap = ConnectClosestRooms(allRooms, connectedMap, passageWidth);
            }

            return connectedMap;
        }

        void CreatePassage(Room roomA, Room roomB, Vector2Int tileA, Vector2Int tileB, int[,] map, int passageWidth)
        {
            Room.ConnectRooms(roomA, roomB);
            // Debug.DrawLine (CoordToWorldPoint (tileA), CoordToWorldPoint (tileB), Color.green, 10);

            List<Vector2Int> line = GetLine(tileA, tileB);
            foreach (Vector2Int c in line)
            {
                MakeRoomOfLine(c, passageWidth, map);
            }
        }

        private void MakeRoomOfLine(Vector2Int c, int r, int[,] map)
        {
            for (int x = -r; x <= r; x++)
            {
                for (int y = -r; y <= r; y++)
                {
                    if (x * x + y * y <= r * r)
                    {
                        int drawX = c.x + x;
                        int drawY = c.y + y;
                        if (IsInMapRange(drawX, drawY, map))
                        {
                            map[drawX, drawY] = ROOM_TYPE;
                        }
                    }
                }
            }
        }

        private List<Vector2Int> GetLine(Vector2Int from, Vector2Int to)
        {
            List<Vector2Int> line = new List<Vector2Int>();

            int x = from.x;
            int y = from.y;

            int dx = to.x - from.x;
            int dy = to.y - from.y;

            bool inverted = false;
            int step = Math.Sign(dx);
            int gradientStep = Math.Sign(dy);

            int longest = Mathf.Abs(dx);
            int shortest = Mathf.Abs(dy);

            if (longest < shortest)
            {
                inverted = true;
                longest = Mathf.Abs(dy);
                shortest = Mathf.Abs(dx);

                step = Math.Sign(dy);
                gradientStep = Math.Sign(dx);
            }

            int gradientAccumulation = longest / 2;
            for (int i = 0; i < longest; i++)
            {
                line.Add(new Vector2Int(x, y));

                if (inverted)
                {
                    y += step;
                }
                else
                {
                    x += step;
                }

                gradientAccumulation += shortest;
                if (gradientAccumulation >= longest)
                {
                    if (inverted)
                    {
                        x += gradientStep;
                    }
                    else
                    {
                        y += gradientStep;
                    }

                    gradientAccumulation -= longest;
                }
            }

            return line;
        }

        // Just used be drawing a line for debugging
        private Vector3 CoordToWorldPoint(Vector2Int tile, int width, int height)
        {
            return new Vector3(-width / 2 + .5f + tile.x, 2, -height / 2 + .5f + tile.y);
        }

        int[,] CreateRandomFillMap(CaveMapConfig config)
        {
            int[,] randomFillMap = new int[config.bitMapWidth, config.bitMapHeight];
            System.Random pseudoRandom = new System.Random(config.randomSeed);

            for (int x = 0; x < config.bitMapWidth; x++)
            {
                for (int y = 0; y < config.bitMapHeight; y++)
                {
                    if (x == 0 || x == config.bitMapWidth - 1 || y == 0 || y == config.bitMapHeight - 1)
                    {
                        randomFillMap[x, y] = WALL_TYPE;
                    }
                    else
                    {
                        randomFillMap[x, y] = (pseudoRandom.Next(0, 100) < config.randomFillPercent)
                            ? WALL_TYPE
                            : ROOM_TYPE;
                    }
                }
            }

            return randomFillMap;
        }

        int[,] SmoothMap(int[,] map, CaveMapConfig config)
        {
            var smoothedMap = map.Clone() as int[,];
            for (int x = 0; x < config.bitMapWidth; x++)
            {
                for (int y = 0; y < config.bitMapHeight; y++)
                {
                    int neighbourWallTiles = GetSurroundingWallCount(x, y, map);

                    if (neighbourWallTiles >= config.neighbourWallsNeededToStayWall)
                        smoothedMap[x, y] = WALL_TYPE;
                    else if (neighbourWallTiles < config.neighbourWallsNeededToStayWall)
                        smoothedMap[x, y] = ROOM_TYPE;
                }
            }

            return smoothedMap;
        }

        int GetSurroundingWallCount(int gridX, int gridY, int[,] map)
        {
            int wallCount = 0;
            for (int neighbourX = gridX - 1; neighbourX <= gridX + 1; neighbourX++)
            {
                for (int neighbourY = gridY - 1; neighbourY <= gridY + 1; neighbourY++)
                {
                    if (IsInMapRange(neighbourX, neighbourY, map))
                    {
                        if ((neighbourX != gridX && neighbourY != gridY) && map[neighbourX, neighbourY] == WALL_TYPE)
                        {
                            wallCount += 1;
                        }
                    }
                    else
                    {
                        wallCount++;
                    }
                }
            }

            return wallCount;
        }
    }
}
