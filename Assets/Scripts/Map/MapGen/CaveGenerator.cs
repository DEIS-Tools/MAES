using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using Quaternion = UnityEngine.Quaternion;
using Random = System.Random;
using Vector3 = UnityEngine.Vector3;

namespace Maes.Map.MapGen
{
    public class CaveGenerator : MapGenerator
    {
        private CaveMapConfig _caveConfig;
        private float _wallHeight;

        /// TODO: Update this to new format
        /// <summary>
        /// Generates a cave map using the unity game objects Plane, InnerWalls and WallRoof.
        /// </summary>
        /// <param name="config">Determines how the cave map should look</param>
        /// <param name="wallHeight">A smaller wall height can make it easier to see the robots. Must be a positive value.</param>
        /// <returns> A SimulationMap represents a map of square tiles, where each tile is divided into 8 triangles as
        /// used in the Marching Squares Algorithm.</returns>
        public void Init(CaveMapConfig config, float wallHeight = 2.0f)
        {
            _caveConfig = config;
            _wallHeight = wallHeight;
            Tile.Rand = new Random(_caveConfig.randomSeed);
        }

        public SimulationMap<bool> GenerateCaveMap()
        {
            // Clear and destroy objects from previous map
            ClearMap();

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
            var smoothedMap = randomlyFilledMap;
            for (var i = 0; i < caveConfig.smoothingRuns; i++)
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
            // MapToDraw = borderedMap;

            // The rooms should now reflect their relative shifted positions after adding borders round map.
            survivingRooms.ForEach(r => r.OffsetCoordsBy(caveConfig.borderSize, caveConfig.borderSize));

            var meshGen = GetComponent<MeshGenerator>();
            var collisionMap = meshGen.GenerateMesh(borderedMap.Clone() as Tile[,], wallHeight,
                false, survivingRooms);

            // Rotate to fit 2D view
            Plane.rotation = Quaternion.AngleAxis(-90, Vector3.right);

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
        private Tile[,] WallOffNarrowCorridors(Tile[,] map)
        {
            var newMap = map.Clone() as Tile[,];
            var tilesToCheck = new Queue<(int, int)>();

            // Populate queue with all tiles
            for (var x = 0; x < newMap!.GetLength(0); x++)
            {
                for (var y = 0; y < newMap.GetLength(1); y++)
                {
                    if (newMap[x, y].Type == TileType.Room)
                        tilesToCheck.Enqueue((x, y));
                }
            }

            while (tilesToCheck.Count > 0)
            {
                var (x, y) = tilesToCheck.Dequeue();
                if (Tile.IsWall(newMap[x, y].Type))
                    continue;

                // Check 3 tiles horizontally
                var horizontalClear = 
                    IsInMapRange(x - 1, y, newMap) && newMap[x - 1, y].Type == TileType.Room && IsInMapRange(x + 1, y, newMap) && newMap[x + 1, y].Type == TileType.Room || 
                    IsInMapRange(x + 1, y, newMap) && newMap[x + 1, y].Type == TileType.Room && IsInMapRange(x + 2, y, newMap) && newMap[x + 2, y].Type == TileType.Room || 
                    IsInMapRange(x - 1, y, newMap) && newMap[x - 1, y].Type == TileType.Room && IsInMapRange(x - 2, y, newMap) && newMap[x - 2, y].Type == TileType.Room;

                // Check 3 tiles vertically
                var verticalClear =
                    IsInMapRange(x, y - 1, newMap) && newMap[x, y - 1].Type == TileType.Room && IsInMapRange(x, y + 1, newMap) && newMap[x, y + 1].Type == TileType.Room ||
                    IsInMapRange(x, y + 1, newMap) && newMap[x, y + 1].Type == TileType.Room && IsInMapRange(x, y + 2, newMap) && newMap[x, y + 2].Type == TileType.Room ||
                    IsInMapRange(x, y - 1, newMap) && newMap[x, y - 1].Type == TileType.Room && IsInMapRange(x, y - 2, newMap) && newMap[x, y - 2].Type == TileType.Room;

                // Check 2 tiles from bottom left to top right clear
                var bottomLeftToTopRightClear =
                    IsInMapRange(x - 1, y - 1, newMap) && newMap[x - 1, y - 1].Type == TileType.Room ||
                    IsInMapRange(x + 1, y + 1, newMap) && newMap[x + 1, y + 1].Type == TileType.Room;

                // Check 2 tiles from top left to bottom right clear
                var topLeftToBottomRightClear =
                    IsInMapRange(x - 1, y + 1, newMap) && newMap[x - 1, y + 1].Type == TileType.Room ||
                    IsInMapRange(x + 1, y - 1, newMap) && newMap[x + 1, y - 1].Type == TileType.Room;

                if (horizontalClear && verticalClear && bottomLeftToTopRightClear && topLeftToBottomRightClear)
                    continue;

                newMap[x, y] = Tile.GetRandomWall();
                // enqueue neighbours to be checked again
                for (var neighborX = x - 1; neighborX < x + 1; neighborX++)
                    for (var neighborY = y - 1; neighborY < y + 1; neighborY++)
                        if (IsInMapRange(neighborX, neighborY, newMap))
                            tilesToCheck.Enqueue((neighborX, neighborY));
            }

            return newMap;
        }

        private Tile[,] ConnectAllRoomsToMainRoom(List<Room> survivingRooms, Tile[,] map, CaveMapConfig config)
        {
            var connectedMap = map.Clone() as Tile[,];
            survivingRooms.Sort();
            survivingRooms[0].IsMainRoom = true;
            survivingRooms[0].IsAccessibleFromMainRoom = true;

            return ConnectClosestRooms(survivingRooms, connectedMap, config.connectionPassagesWidth);
        }

        private Tile[,] ConnectClosestRooms(List<Room> allRooms, Tile[,] map, int passageWidth)
        {
            var connectedMap = map.Clone() as Tile[,];
            var roomListA = new List<Room>();
            var roomListB = new List<Room>();


            foreach (var room in allRooms)
            {
                if (room.IsAccessibleFromMainRoom)
                    roomListB.Add(room);
                else
                    roomListA.Add(room);
            }


            var bestDistance = 0;
            var bestTileA = new Vector2Int();
            var bestTileB = new Vector2Int();
            var bestRoomA = new Room();
            var bestRoomB = new Room();
            var possibleConnectionFound = false;

            foreach (var roomA in roomListA)
            {
                foreach (var roomB in roomListB.Where(roomB => roomA != roomB && !roomA.IsConnected(roomB)))
                {
                    foreach (var tileA in roomA.EdgeTiles)
                    {
                        foreach (var tileB in roomB.EdgeTiles)
                        {
                            var distanceBetweenRooms =
                                (int)(Mathf.Pow(tileA.x - tileB.x, 2) + Mathf.Pow(tileA.y - tileB.y, 2));

                            if (distanceBetweenRooms >= bestDistance && possibleConnectionFound) 
                                continue;

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

            if (!possibleConnectionFound) 
                return connectedMap;

            CreatePassage(bestRoomA, bestRoomB, bestTileA, bestTileB, connectedMap, passageWidth);
            connectedMap = ConnectClosestRooms(allRooms, connectedMap, passageWidth);

            return connectedMap;
        }

        void CreatePassage(Room roomA, Room roomB, Vector2Int tileA, Vector2Int tileB, Tile[,] map, int passageWidth)
        {
            Room.ConnectRooms(roomA, roomB);
            //Debug.DrawLine(CoordToWorldPoint(tileA, map.GetLength(0), map.GetLength(1)),
            //    CoordToWorldPoint(tileB, map.GetLength(0), map.GetLength(1)), 
            //    Color.green,
            //    100);

            var line = GetLine(tileA, tileB);
            foreach (var c in line)
            {
                MakeRoomOfLine(c, passageWidth, map);
            }
        }

        private void MakeRoomOfLine(Vector2Int c, int r, Tile[,] map)
        {
            for (var x = -r; x <= r; x++)
            {
                for (var y = -r; y <= r; y++)
                {
                    if (x * x + y * y > r * r) 
                        continue;

                    var drawX = c.x + x;
                    var drawY = c.y + y;
                    if (IsInMapRange(drawX, drawY, map))
                    {
                        map[drawX, drawY] = new Tile(TileType.Room);
                    }
                }
            }
        }

        private static List<Vector2Int> GetLine(Vector2Int from, Vector2Int to)
        {
            var line = new List<Vector2Int>();

            var x = from.x;
            var y = from.y;

            var dx = to.x - from.x;
            var dy = to.y - from.y;

            var inverted = false;
            var step = Math.Sign(dx);
            var gradientStep = Math.Sign(dy);

            var longest = Mathf.Abs(dx);
            var shortest = Mathf.Abs(dy);

            if (longest < shortest)
            {
                inverted = true;
                longest = Mathf.Abs(dy);
                shortest = Mathf.Abs(dx);

                step = Math.Sign(dy);
                gradientStep = Math.Sign(dx);
            }

            var gradientAccumulation = longest / 2;
            for (var i = 0; i < longest; i++)
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
                if (gradientAccumulation < longest) 
                    continue;

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

            return line;
        }

        // Just used be drawing a line for debugging
        private Vector3 CoordToWorldPoint(Vector2Int tile, int width, int height)
        {
            return new Vector3(-width / 2 + .5f + tile.x, -height / 2 + .5f + tile.y, 2);
        }

        private static Tile[,] CreateRandomFillMap(CaveMapConfig config)
        {
            var randomFillMap = new Tile[config.bitMapWidth, config.bitMapHeight];
            var pseudoRandom = new Random(config.randomSeed);

            for (var x = 0; x < config.bitMapWidth; x++)
            {
                for (var y = 0; y < config.bitMapHeight; y++)
                {
                    if (x == 0 || x == config.bitMapWidth - 1 || y == 0 || y == config.bitMapHeight - 1)
                        randomFillMap[x, y] = Tile.GetRandomWall();
                    else
                        randomFillMap[x, y] = pseudoRandom.Next(0, 100) < config.randomFillPercent
                            ? Tile.GetRandomWall()
                            : new Tile(TileType.Room);
                }
            }

            return randomFillMap;
        }

        private Tile[,] SmoothMap(Tile[,] map, CaveMapConfig config)
        {
            var smoothedMap = map.Clone() as Tile[,];
            for (var x = 0; x < config.bitMapWidth; x++)
            {
                for (var y = 0; y < config.bitMapHeight; y++)
                {
                    var (neighborWallTiles, neighborWallType) = GetSurroundingWallCount(x, y, map);

                    if (neighborWallTiles >= config.neighbourWallsNeededToStayWall)
                        smoothedMap![x, y] = new Tile(neighborWallType);
                    else
                        smoothedMap![x, y] = new Tile(TileType.Room);
                }
            }

            return smoothedMap;
        }

        private (int, TileType) GetSurroundingWallCount(int gridX, int gridY, Tile[,] map)
        {
            var wallCount = 0;
            var wallTypes = new Dictionary<TileType,int>();
            for (var neighborX = gridX - 1; neighborX <= gridX + 1; neighborX++)
            {
                for (var neighborY = gridY - 1; neighborY <= gridY + 1; neighborY++)
                {
                    if (IsInMapRange(neighborX, neighborY, map))
                    {
                        if ((neighborX == gridX || neighborY == gridY) || !Tile.IsWall(map[neighborX, neighborY].Type))
                            continue;
                        wallCount += 1;
                        wallTypes[map[neighborX, neighborY].Type] = wallTypes.GetValueOrDefault(map[neighborX, neighborY].Type)+1;
                    }
                    else
                    {
                        wallCount++;
                        var tile = Tile.GetRandomWall();
                        wallTypes[tile.Type] = wallTypes.GetValueOrDefault(tile.Type)+1;
                    }

                }
            }
            
            var mostCommonType = wallCount > 0 ? wallTypes.Aggregate((l,r) => l.Value > r.Value ? l : r).Key : TileType.Room;

            return (wallCount, mostCommonType);
        }
    }
}
