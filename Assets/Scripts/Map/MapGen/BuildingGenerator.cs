using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using Random = System.Random;
using static Maes.Map.MapGen.BitMapTypes;

namespace Maes.Map.MapGen
{
    public class BuildingGenerator : MapGenerator
    {
        private BuildingMapConfig _config;
        private float _wallHeight;

        public void Init(BuildingMapConfig config, float wallHeight = 2.0f)
        {
            _config = config;
            _wallHeight = wallHeight;
        }

        /// <summary>
        /// Generates a building map using the unity game objects Plane, InnerWalls and WallRoof.
        /// </summary>
        /// <param name="config">Determines how the map is generated</param>
        /// <param name="wallHeight">A lower height can make it easier to see the robots. Must be a positive value.</param>
        /// <returns> A SimulationMap represents a map of square tiles, where each tile is divided into 8 triangles as
        /// used in the Marching Squares Algorithm.</returns>
        public SimulationMap<bool> GenerateBuildingMap()
        {
            // Clear and destroy objects from previous map
            clearMap();

            var random = new Random(_config.randomSeed);

            var emptyMap = GenerateEmptyMap(_config.bitMapWidth, _config.bitMapHeight);

            var mapWithHalls = GenerateMapWithHalls(emptyMap, _config, random);

            var mapWithWallsAroundRooms = AddWallAroundRoomRegions(mapWithHalls, _config);

            var mapWithRooms = GenerateRoomsBetweenHalls(mapWithWallsAroundRooms, _config, random);

            var closedHallwayMap = CloseOffHallwayEnds(mapWithRooms);

            // Rooms and halls sorted according to biggest hall first. Biggest hall set to main room.
            // If both rooms are halls, sort according to size
            var rooms = GetSortedRooms(closedHallwayMap);

            var connectedMap = ConnectRoomsWithDoors(rooms, closedHallwayMap, random, _config);

            var borderedMap = CreateBorderedMap(connectedMap, _config.bitMapWidth, _config.bitMapHeight, _config.borderSize);

            // For debugging
            // mapToDraw = borderedMap;

            // The rooms should now reflect their relative shifted positions after adding borders round map.
            rooms.ForEach(r => r.OffsetCoordsBy(_config.borderSize, _config.borderSize));
            var meshGen = GetComponent<MeshGenerator>();
            var collisionMap = meshGen.GenerateMesh(borderedMap.Clone() as int[,], _wallHeight, true,
                rooms);

            // Rotate to fit 2D view
            _plane.rotation = Quaternion.AngleAxis(-90, Vector3.right);

            ResizePlaneToFitMap(_config.bitMapHeight, _config.bitMapWidth);

            MovePlaneAndWallRoofToFitWallHeight(_wallHeight);

            return collisionMap;
        }

        private int[,] ConnectRoomsWithDoors(List<Room> sortedRooms, int[,] oldMap, Random random,
            BuildingMapConfig config)
        {
            int[,] connectedMap = oldMap.Clone() as int[,];
            // Whether the given room is connected to the main room
            var connectedRooms = new List<Room>();
            var nonConnectedRooms = new List<Room>();

            foreach (var room in sortedRooms)
            {
                if (room.isAccessibleFromMainRoom)
                    connectedRooms.Add(room);
                else
                    nonConnectedRooms.Add(room);
            }

            if (nonConnectedRooms.Count == 0)
                return connectedMap;

            foreach (Room cRoom in connectedRooms)
            {
                foreach (var nRoom in nonConnectedRooms)
                {
                    if (cRoom == nRoom || cRoom.IsConnected(nRoom))
                        continue;

                    // If they share any wall, they must be adjacent
                    var sharedWallTiles = cRoom.GetSharedWallTiles(nRoom);
                    if (sharedWallTiles.Count > 0)
                    {
                        int biggestXValue, smallestXValue;
                        int biggestYValue, smallestYValue;

                        // The maxima of x and y are needed to isolate the coordinates for each line of wall
                        smallestXValue = sharedWallTiles.Aggregate((agg, next) => next.x < agg.x ? next : agg).x;
                        biggestXValue = sharedWallTiles.Aggregate((agg, next) => next.x > agg.x ? next : agg).x;
                        smallestYValue = sharedWallTiles.Aggregate((agg, next) => next.y < agg.y ? next : agg).y;
                        biggestYValue = sharedWallTiles.Aggregate((agg, next) => next.y > agg.y ? next : agg).y;

                        List<Vector2Int> line;
                        int maxDoors = random.Next(1, 4);
                        if (smallestYValue == biggestYValue || smallestXValue == biggestXValue)
                            maxDoors = 1;
                        int doorsMade = 0;
                        int doorPadding = config.doorPadding;

                        // A shared wall with the smallest y value
                        if (doorsMade < maxDoors)
                        {
                            // A shared line/wall in the bottom
                            line = sharedWallTiles.FindAll(c => c.y == smallestYValue).ToList();
                            if (line.Count > config.doorWidth + (doorPadding * 2))
                            {
                                line.Sort((c1, c2) => c1.x - c2.x); // Ascending order
                                int doorStartingIndex = random.Next(0 + doorPadding,
                                    (line.Count - 1) - (int)config.doorWidth - doorPadding);
                                var doorStartingPoint = line[doorStartingIndex];
                                for (int i = 0; i < config.doorWidth; i++)
                                {
                                    connectedMap[doorStartingPoint.x + i, doorStartingPoint.y] = ROOM_TYPE;
                                }

                                Room.ConnectRooms(cRoom, nRoom);
                                doorsMade++;
                            }
                        }

                        // Shared wall with the biggest y value
                        if (doorsMade < maxDoors)
                        {
                            // A shared line/wall in the top
                            line = sharedWallTiles.FindAll(c => c.y == biggestYValue).ToList();
                            if (line.Count > config.doorWidth + (doorPadding * 2))
                            {
                                line.Sort((c1, c2) => c1.x - c2.x); // Ascending order
                                int doorStartingIndex = random.Next(0 + doorPadding,
                                    (line.Count - 1) - (int)config.doorWidth - doorPadding);
                                var doorStartingPoint = line[doorStartingIndex];
                                for (int i = 0; i < config.doorWidth; i++)
                                {
                                    connectedMap[doorStartingPoint.x + i, doorStartingPoint.y] = ROOM_TYPE;
                                }

                                Room.ConnectRooms(cRoom, nRoom);
                                doorsMade++;
                            }
                        }

                        // A shared wall with the smallest x value
                        if (doorsMade < maxDoors)
                        {
                            // A shared line/wall on the left
                            line = sharedWallTiles.FindAll(c => c.x == smallestXValue).ToList();
                            if (line.Count > config.doorWidth + (doorPadding * 2))
                            {
                                line.Sort((c1, c2) => c1.y - c2.y); // Ascending order
                                int doorStartingIndex = random.Next(0 + doorPadding,
                                    (line.Count - 1) - (int)config.doorWidth - doorPadding);
                                var doorStartingPoint = line[doorStartingIndex];
                                for (int i = 0; i < config.doorWidth; i++)
                                {
                                    connectedMap[doorStartingPoint.x, doorStartingPoint.y + i] = ROOM_TYPE;
                                }

                                Room.ConnectRooms(cRoom, nRoom);
                                doorsMade++;
                            }
                        }

                        // A shared wall with the biggest x value
                        if (doorsMade < maxDoors)
                        {
                            // A shared line/wall on the right
                            line = sharedWallTiles.FindAll(c => c.x == biggestXValue).ToList();
                            if (line.Count > config.doorWidth + (doorPadding * 2))
                            {
                                line.Sort((c1, c2) => c1.y - c2.y); // Ascending order
                                int doorStartingIndex = random.Next(0 + doorPadding,
                                    (line.Count - 1) - (int)config.doorWidth - doorPadding);
                                var doorStartingPoint = line[doorStartingIndex];
                                for (int i = 0; i < config.doorWidth; i++)
                                {
                                    connectedMap[doorStartingPoint.x, doorStartingPoint.y + i] = ROOM_TYPE;
                                }

                                Room.ConnectRooms(cRoom, nRoom);
                                doorsMade++;
                            }
                        }
                    }
                }
            }

            // Recursive call that continues, until all rooms and halls are connected
            connectedMap = ConnectRoomsWithDoors(sortedRooms, connectedMap, random, config);

            return connectedMap;
        }

        private List<Room> GetSortedRooms(int[,] map)
        {
            List<List<Vector2Int>> roomRegions = GetRegions(map, ROOM_TYPE, HALL_TYPE);

            List<Room> rooms = new List<Room>();
            foreach (var region in roomRegions)
            {
                rooms.Add(new Room(region, map));
            }

            // Sort by first tiletype = Hall, then size
            // Descending order. Hallway > other room types
            rooms.Sort((o1, o2) => {
                var o1x = o1.tiles[0].x;
                var o1y = o1.tiles[0].y;
                var o2x = o2.tiles[0].x;
                var o2y = o2.tiles[0].y;
                if (map[o1x, o1y] != map[o2x, o2y])
                    if (map[o1x, o1y] == HALL_TYPE && map[o2x, o2y] != HALL_TYPE)
                        return -1;
                    else if (map[o1x, o1y] != HALL_TYPE && map[o2x, o2y] == HALL_TYPE)
                        return 1;

                return o2.roomSize - o1.roomSize;
            });
            // This should now be the biggest hallway
            rooms[0].isMainRoom = true;
            rooms[0].isAccessibleFromMainRoom = true;

            return rooms;
        }

        private int[,] CloseOffHallwayEnds(int[,] oldMap)
        {
            var map = oldMap.Clone() as int[,];

            var mapWidth = map.GetLength(0);
            var mapHeight = map.GetLength(1);

            for (int x = 0; x < mapWidth; x++)
            {
                for (int y = 0; y < mapHeight; y++)
                {
                    if (x == mapWidth - 1 || x == 0 || y == 0 || y == mapHeight - 1)
                    {
                        map[x, y] = WALL_TYPE;
                    }
                }
            }

            return map;
        }

        private int[,] GenerateRoomsBetweenHalls(int[,] oldMap, BuildingMapConfig config, Random random)
        {
            var mapWithRooms = oldMap.Clone() as int[,];

            List<List<Vector2Int>> roomRegions = GetRegions(mapWithRooms, ROOM_TYPE);

            foreach (List<Vector2Int> roomRegion in roomRegions)
            {
                SplitRoomRegion(mapWithRooms, roomRegion, config, false, true, random);
            }

            return mapWithRooms;
        }

        // This function has side effects on the map!
        private void SplitRoomRegion(int[,] map, List<Vector2Int> roomRegion, BuildingMapConfig config, bool splitOnXAxis,
            bool forceSplit, Random random)
        {
            // Check if we want to split. This allows for different size rooms
            bool shouldSplit = random.Next(0, 100) <= config.roomSplitChancePercent;

            if (!shouldSplit && !forceSplit)
                return;

            // Find where to split
            // Rotate 90 degrees every time an room is split
            // We either use the x axis or y axis as starting point
            List<int> coords;
            if (splitOnXAxis)
                coords = roomRegion.Select(coord => coord.x).ToList();
            else
                coords = roomRegion.Select(coord => coord.y).ToList();

            // Filter out coords that would be too close to the edges to allow for 2 rooms side by side according to config.minRoomSideLength
            var sortedCoords = coords.OrderBy(c => c).ToList();
            var smallestValue = sortedCoords[0];
            var biggestValue = sortedCoords[sortedCoords.Count - 1];

            // +1 to allow for wall between room spaces
            var filteredCoords = sortedCoords.FindAll(x =>
                x + config.minRoomSideLength < biggestValue - config.minRoomSideLength && // Right side
                x > smallestValue + config.minRoomSideLength
            );

            // If no possible split on the current axis is possible, try the other one with guaranteed split
            if (filteredCoords.Count == 0)
            {
                // Force split only gets once chance to avoid stack overflow
                if (!forceSplit)
                {
                    SplitRoomRegion(map, roomRegion, config, !splitOnXAxis, true, random);
                }

                return;
            }

            // Select random index to start the wall
            var selectedIndex = random.Next(filteredCoords.Count);
            var wallStartCoord = filteredCoords[selectedIndex];

            // Create wall
            if (splitOnXAxis)
            {
                foreach (var c in roomRegion)
                {
                    if (c.x == wallStartCoord)
                        map[c.x, c.y] = WALL_TYPE;
                }
            }
            else
            {
                foreach (var c in roomRegion)
                {
                    if (c.y == wallStartCoord)
                        map[c.x, c.y] = WALL_TYPE;
                }
            }

            // Get two new regions
            List<Vector2Int> newRegion1, newRegion2;
            Vector2Int region1Tile, region2Tile; // A random tile from each region. We use flooding algorithm to find the rest
            if (splitOnXAxis)
            {
                region1Tile = roomRegion.Find(c => c.x < wallStartCoord);
                region2Tile = roomRegion.Find(c => c.x > wallStartCoord);
            }
            else
            {
                region1Tile = roomRegion.Find(c => c.y < wallStartCoord);
                region2Tile = roomRegion.Find(c => c.y > wallStartCoord);
            }

            newRegion1 = GetRegionTiles(region1Tile.x, region1Tile.y, map);
            newRegion2 = GetRegionTiles(region2Tile.x, region2Tile.y, map);

            // Run function recursively
            SplitRoomRegion(map, newRegion1, config, !splitOnXAxis, false, random);
            SplitRoomRegion(map, newRegion2, config, !splitOnXAxis, false, random);
        }

        private int[,] AddWallAroundRoomRegions(int[,] oldMap, BuildingMapConfig config)
        {
            var mapWithRoomWalls = oldMap.Clone() as int[,];

            List<List<Vector2Int>> roomRegions = GetRegions(mapWithRoomWalls, ROOM_TYPE);

            foreach (var region in roomRegions)
            {
                // Get smallest and largest x and y
                var sortedXsAsc = region.Select(c => c.x).ToList().OrderBy(x => x).ToList();
                var sortedYsAsc = region.Select(c => c.y).ToList().OrderBy(y => y).ToList();

                var smallestX = sortedXsAsc[0];
                var biggestX = sortedXsAsc[sortedXsAsc.Count - 1];
                var smallestY = sortedYsAsc[0];
                var biggestY = sortedYsAsc[sortedYsAsc.Count - 1];

                foreach (var coord in region)
                {
                    if (coord.x == smallestX || coord.x == biggestX || coord.y == smallestY || coord.y == biggestY)
                    {
                        mapWithRoomWalls[coord.x, coord.y] = WALL_TYPE;
                    }
                }
            }

            return mapWithRoomWalls;
        }

        private int[,] GenerateMapWithHalls(int[,] map, BuildingMapConfig config, Random random)
        {
            var mapWithHalls = map.Clone() as int[,];

            // Rotate 90 degrees every time a hallway is generated
            bool usingXAxis = true;
            while (GetHallPercentage(mapWithHalls) < config.maxHallInPercent)
            {
                // We always split the currently biggest room
                List<List<Vector2Int>> roomRegions = GetRegions(mapWithHalls, ROOM_TYPE);
                List<Vector2Int> biggestRoom = roomRegions.OrderByDescending(l => l.Count).ToList()[0];

                // We either use the x axis or y axis as starting point
                List<int> coords;
                if (usingXAxis)
                    coords = biggestRoom.Select(coord => coord.x).ToList();
                else
                    coords = biggestRoom.Select(coord => coord.y).ToList();

                // Filter out coords too close to the edges to be a hall
                var sortedCoords = coords.OrderBy(c => c).ToList();
                var smallestValue = sortedCoords[0];
                var biggestValue = sortedCoords[sortedCoords.Count - 1];
                // Plus 2 to allow for inner walls in the room spaces
                var filteredCoords = sortedCoords.FindAll(x =>
                    x + config.minRoomSideLength + 2 < biggestValue - config.minRoomSideLength + 2 && // Right side
                    x > smallestValue + config.minRoomSideLength + 2
                );

                if (filteredCoords.Count == 0)
                {
                    Debug.Log("The maxHallInPercent of " + config.maxHallInPercent + " could not be achieved.");
                    break;
                }

                // Select random index to fill with hallway
                var selectedIndex = random.Next(filteredCoords.Count);
                var hallStartCoord = filteredCoords[selectedIndex];

                if (usingXAxis)
                {
                    // Fill with hall tiles
                    foreach (var c in biggestRoom)
                    {
                        if (hallStartCoord <= c.x && c.x < hallStartCoord + config.hallWidth)
                        {
                            mapWithHalls[c.x, c.y] = HALL_TYPE;
                        }
                    }

                    usingXAxis = false;
                }
                else
                {
                    // Fill with hall tiles
                    foreach (var c in biggestRoom)
                    {
                        if (hallStartCoord <= c.y && c.y < hallStartCoord + config.hallWidth)
                        {
                            mapWithHalls[c.x, c.y] = HALL_TYPE;
                        }
                    }

                    usingXAxis = true;
                }
            }

            return mapWithHalls;
        }

        private int[,] GenerateEmptyMap(int width, int height)
        {
            int[,] emptyMap = new int[width, height];

            for (int x = 0; x < emptyMap.GetLength(0); x++)
            {
                for (int y = 0; y < emptyMap.GetLength(1); y++)
                {
                    emptyMap[x, y] = ROOM_TYPE;
                }
            }

            return emptyMap;
        }

        private float GetHallPercentage(int[,] map)
        {
            int width = map.GetLength(0);
            int height = map.GetLength(1);
            int mapSize = width * height;

            int amountOfHall = 0;
            for (int x = 0; x < width; x++)
            {
                for (int y = 0; y < height; y++)
                {
                    amountOfHall += map[x, y] == HALL_TYPE ? 1 : 0;
                }
            }

            return (amountOfHall / (float)mapSize) * 100f;
        }

    }
}