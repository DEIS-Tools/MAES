using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using System;
using System.Globalization;
using System.Linq;
using System.Linq.Expressions;
using System.Numerics;
using System.Threading;
using Dora.MapGeneration;
using UnityEditor.UI;
using Quaternion = UnityEngine.Quaternion;
using Random = System.Random;
using Vector3 = UnityEngine.Vector3;


public class MapGenerator : MonoBehaviour {
    public Transform plane;
    public Transform innerWalls;
    public Transform wallRoof;

    public MeshGenerator meshGenerator;

    // Variable used for drawing gizmos on selection for debugging.
    private int[,] mapToDraw = null;

    void Update() {
        if (Input.GetButtonUp("Jump")) {
            /*var config = new CaveMapConfig(100,
                100,
                Time.time.ToString(),
                4,
                2,
                48,
                10,
                1,
                1,
                2);
            var map = GenerateCaveMap(config, 
                                        3.0f,
                                        true);*/
            /*var officeConfig = new OfficeMapConfig(60, 60, Time.time.ToString(), 8, 3, 5, 2, 0, 65, 2, 2.0f);
            var map = GenerateOfficeMap(officeConfig, 3.0f, true);*/
        }


        /*if (Input.GetMouseButtonDown(1) && (Input.GetKey(KeyCode.LeftControl) || Input.GetKey(KeyCode.RightControl)))
        {
            clearMap();
        }*/
    }

    private void MovePlaneAndWallRoofToFitWallHeight(float wallHeight, bool is2D = true) {
        // Move walls and wall roof to above plane depending on wall height
        // The axis depends on whether it is 3D or 2D.
        if (is2D) {
            Vector3 newPosition = wallRoof.position;
            newPosition.z = -wallHeight;
            wallRoof.position = newPosition;

            newPosition = innerWalls.position;
            newPosition.z = -wallHeight;
            innerWalls.position = newPosition;
        }
        else {
            Vector3 newPosition = wallRoof.position;
            newPosition.y = wallHeight;
            wallRoof.position = newPosition;

            newPosition = innerWalls.position;
            newPosition.y = wallHeight;
            innerWalls.position = newPosition;
        }
    }

    private void ResizePlaneToFitMap(int bitMapHeight, int bitMapWidth, float scaling, float padding = 0.1f) {
        // Resize plane below cave to fit size
        plane.localScale = new Vector3(((bitMapWidth * scaling) / 10f) + padding,
            1,
            ((bitMapHeight * scaling) / 10f) + padding);
    }

    public void clearMap() {
        meshGenerator.ClearMesh();
    }

    /**
	 * Methods for creating office map
	 */
    public SimulationMap<bool> GenerateOfficeMap(OfficeMapConfig config, float wallHeight, bool is2D = true) {
        // Clear and destroy objects from previous map
        clearMap();

        Random random = new Random(config.randomSeed);

        int[,] emptyMap = GenerateEmptyMap(config.bitMapWidth, config.bitMapHeight);

        var mapWithHalls = GenerateMapWithHalls(emptyMap, config, random);

        var mapWithWallsAroundOffices = AddWallAroundOfficeRegions(mapWithHalls, config);

        var mapWithOfficeRooms = GenerateOfficesBetweenHalls(mapWithWallsAroundOffices, config, random);

        var closedHallwayMap = CloseOffHallwayEnds(mapWithOfficeRooms);

        // Offices and halls sorted according to biggest hall first. Biggest hall set to main room.
        // If both rooms are halls, sort according to size
        var offices = GetSortedOfficeRooms(closedHallwayMap);

        var connectedMap = ConnectOfficesWithDoors(offices, closedHallwayMap, random, config);

        var borderedMap = CreateBorderedMap(connectedMap, config.bitMapWidth, config.bitMapHeight, config.borderSize);

        // For debugging
        // mapToDraw = borderedMap;

        // The rooms should now reflect their relative shifted positions after adding borders round map.
        offices.ForEach(r => r.OffsetCoordsBy(config.borderSize, config.borderSize));
        MeshGenerator meshGen = GetComponent<MeshGenerator>();
        var collisionMap = meshGen.GenerateMesh(borderedMap.Clone() as int[,], config.scaling, wallHeight, is2D, true,
            offices);

        if (is2D) {
            plane.rotation = Quaternion.AngleAxis(-90, Vector3.right);
        }

        ResizePlaneToFitMap(config.bitMapHeight, config.bitMapWidth, config.scaling);

        MovePlaneAndWallRoofToFitWallHeight(wallHeight, is2D);

        return collisionMap;
    }

    private int[,] ConnectOfficesWithDoors(List<Room> sortedOffices, int[,] oldMap, Random random,
        OfficeMapConfig config) {
        int[,] connectedMap = oldMap.Clone() as int[,];
        // Whether the given room is connected to the main room
        List<Room> connectedRooms = new List<Room>();
        List<Room> nonConnectedRooms = new List<Room>();

        foreach (var room in sortedOffices) {
            if (room.isAccessibleFromMainRoom)
                connectedRooms.Add(room);
            else
                nonConnectedRooms.Add(room);
        }

        if (nonConnectedRooms.Count == 0)
            return connectedMap;

        foreach (Room cRoom in connectedRooms) {
            foreach (var nRoom in nonConnectedRooms) {
                if (cRoom == nRoom || cRoom.IsConnected(nRoom))
                    continue;

                // If they share any wall, they must be adjacent
                var sharedWallTiles = cRoom.GetSharedWallTiles(nRoom);
                if (sharedWallTiles.Count > 0) {
                    int biggestXValue, smallestXValue;
                    int biggestYValue, smallestYValue;

                    // The maxima of x and y are needed to isolate the coordinates for each line of wall
                    sharedWallTiles.Sort((c1, c2) => c1.x - c2.x);
                    smallestXValue = sharedWallTiles[0].x;
                    biggestXValue = sharedWallTiles[sharedWallTiles.Count - 1].x;
                    sharedWallTiles.Sort((c1, c2) => c1.y - c2.y);
                    smallestYValue = sharedWallTiles[0].y;
                    biggestYValue = sharedWallTiles[sharedWallTiles.Count - 1].y;

                    List<Coord> line;
                    int maxDoors = random.Next(1, 4);
                    if (smallestYValue == biggestYValue || smallestXValue == biggestXValue)
                        maxDoors = 1;
                    int doorsMade = 0;
                    int doorPadding = config.doorPadding;

                    // A shared wall with the smallest y value
                    if (doorsMade < maxDoors) {
                        // A shared line/wall in the bottom
                        line = sharedWallTiles.FindAll(c => c.y == smallestYValue).ToList();
                        if (line.Count > config.doorWidth + (doorPadding * 2)) {
                            line.Sort((c1, c2) => c1.x - c2.x); // Ascending order
                            int doorStartingIndex = random.Next(0 + doorPadding,
                                (line.Count - 1) - (int) config.doorWidth - doorPadding);
                            var doorStartingPoint = line[doorStartingIndex];
                            for (int i = 0; i < config.doorWidth; i++) {
                                connectedMap[doorStartingPoint.x + i, doorStartingPoint.y] = BitMapTypes.ROOM_TYPE;
                            }

                            Room.ConnectRooms(cRoom, nRoom);
                            doorsMade++;
                        }
                    }

                    // Shared wall with the biggest y value
                    if (doorsMade < maxDoors) {
                        // A shared line/wall in the top
                        line = sharedWallTiles.FindAll(c => c.y == biggestYValue).ToList();
                        if (line.Count > config.doorWidth + (doorPadding * 2)) {
                            line.Sort((c1, c2) => c1.x - c2.x); // Ascending order
                            int doorStartingIndex = random.Next(0 + doorPadding,
                                (line.Count - 1) - (int) config.doorWidth - doorPadding);
                            var doorStartingPoint = line[doorStartingIndex];
                            for (int i = 0; i < config.doorWidth; i++) {
                                connectedMap[doorStartingPoint.x + i, doorStartingPoint.y] = BitMapTypes.ROOM_TYPE;
                            }

                            Room.ConnectRooms(cRoom, nRoom);
                            doorsMade++;
                        }
                    }

                    // A shared wall with the smallest x value
                    if (doorsMade < maxDoors) {
                        // A shared line/wall on the left
                        line = sharedWallTiles.FindAll(c => c.x == smallestXValue).ToList();
                        if (line.Count > config.doorWidth + (doorPadding * 2)) {
                            line.Sort((c1, c2) => c1.y - c2.y); // Ascending order
                            int doorStartingIndex = random.Next(0 + doorPadding,
                                (line.Count - 1) - (int) config.doorWidth - doorPadding);
                            var doorStartingPoint = line[doorStartingIndex];
                            for (int i = 0; i < config.doorWidth; i++) {
                                connectedMap[doorStartingPoint.x, doorStartingPoint.y + i] = BitMapTypes.ROOM_TYPE;
                            }

                            Room.ConnectRooms(cRoom, nRoom);
                            doorsMade++;
                        }
                    }

                    // A shared wall with the biggest x value
                    if (doorsMade < maxDoors) {
                        // A shared line/wall on the right
                        line = sharedWallTiles.FindAll(c => c.x == biggestXValue).ToList();
                        if (line.Count > config.doorWidth + (doorPadding * 2)) {
                            line.Sort((c1, c2) => c1.y - c2.y); // Ascending order
                            int doorStartingIndex = random.Next(0 + doorPadding,
                                (line.Count - 1) - (int) config.doorWidth - doorPadding);
                            var doorStartingPoint = line[doorStartingIndex];
                            for (int i = 0; i < config.doorWidth; i++) {
                                connectedMap[doorStartingPoint.x, doorStartingPoint.y + i] = BitMapTypes.ROOM_TYPE;
                            }

                            Room.ConnectRooms(cRoom, nRoom);
                            doorsMade++;
                        }
                    }
                }
            }
        }

        // Recursive call that continues, until all offices and halls are connected
        connectedMap = ConnectOfficesWithDoors(sortedOffices, connectedMap, random, config);

        return connectedMap;
    }

    private List<Room> GetSortedOfficeRooms(int[,] map) {
        List<List<Coord>> roomRegions = GetRegions(map, BitMapTypes.ROOM_TYPE, BitMapTypes.HALL_TYPE);

        List<Room> offices = new List<Room>();
        foreach (var region in roomRegions) {
            offices.Add(new Room(region, map));
        }

        // Sort by first tiletype = Hall, then size
        // Descending order. Hallway > other room types
        offices.Sort((o1, o2) => {
            var o1x = o1.tiles[0].x;
            var o1y = o1.tiles[0].y;
            var o2x = o2.tiles[0].x;
            var o2y = o2.tiles[0].y;
            if (map[o1x, o1y] != map[o2x, o2y])
                if (map[o1x, o1y] == BitMapTypes.HALL_TYPE && map[o2x, o2y] != BitMapTypes.HALL_TYPE)
                    return -1;
                else if (map[o1x, o1y] != BitMapTypes.HALL_TYPE && map[o2x, o2y] == BitMapTypes.HALL_TYPE)
                    return 1;

            return o2.roomSize - o1.roomSize;
        });
        // This should now be the biggest hallway
        offices[0].isMainRoom = true;
        offices[0].isAccessibleFromMainRoom = true;

        return offices;
    }

    private int[,] CloseOffHallwayEnds(int[,] oldMap) {
        var map = oldMap.Clone() as int[,];

        var mapWidth = map.GetLength(0);
        var mapHeight = map.GetLength(1);

        for (int x = 0; x < mapWidth; x++) {
            for (int y = 0; y < mapHeight; y++) {
                if (x == mapWidth - 1 || x == 0 || y == 0 || y == mapHeight - 1) {
                    map[x, y] = BitMapTypes.WALL_TYPE;
                }
            }
        }

        return map;
    }

    private int[,] GenerateOfficesBetweenHalls(int[,] oldMap, OfficeMapConfig config, Random random) {
        var mapWithOffices = oldMap.Clone() as int[,];

        List<List<Coord>> officeRegions = GetRegions(mapWithOffices, BitMapTypes.ROOM_TYPE);

        foreach (List<Coord> officeRegion in officeRegions) {
            SplitOfficeRegion(mapWithOffices, officeRegion, config, false, true, random);
        }

        return mapWithOffices;
    }

    // This function has side effects on the map!
    private void SplitOfficeRegion(int[,] map, List<Coord> officeRegion, OfficeMapConfig config, bool splitOnXAxis,
        bool forceSplit, Random random) {
        // Check if we want to split. This allows for different size offices
        bool shouldSplit = random.Next(0, 100) <= config.officeSplitChancePercent;

        if (!shouldSplit && !forceSplit)
            return;

        // Find where to split
        // Rotate 90 degrees every time an office is split
        // We either use the x axis or y axis as starting point
        List<int> coords;
        if (splitOnXAxis)
            coords = officeRegion.Select(coord => coord.x).ToList();
        else
            coords = officeRegion.Select(coord => coord.y).ToList();

        // Filter out coords that would be too close to the edges to allow for 2 rooms side by side according to config.minRoomSideLength
        var sortedCoords = coords.OrderBy(c => c).ToList();
        var smallestValue = sortedCoords[0];
        var biggestValue = sortedCoords[sortedCoords.Count - 1];

        // +1 to allow for wall between office spaces
        var filteredCoords = sortedCoords.FindAll(x =>
            x + config.minRoomSideLength < biggestValue - config.minRoomSideLength && // Right side
            x > smallestValue + config.minRoomSideLength
        );

        // If no possible split on the current axis is possible, try the other one with guaranteed split
        if (filteredCoords.Count == 0) {
            // Force split only gets once chance to avoid stack overflow
            if (!forceSplit) {
                SplitOfficeRegion(map, officeRegion, config, !splitOnXAxis, true, random);
            }

            return;
        }

        // Select random index to start the wall
        var selectedIndex = random.Next(filteredCoords.Count);
        var wallStartCoord = filteredCoords[selectedIndex];

        // Create wall
        if (splitOnXAxis) {
            foreach (var c in officeRegion) {
                if (c.x == wallStartCoord)
                    map[c.x, c.y] = BitMapTypes.WALL_TYPE;
            }
        }
        else {
            foreach (var c in officeRegion) {
                if (c.y == wallStartCoord)
                    map[c.x, c.y] = BitMapTypes.WALL_TYPE;
            }
        }

        // Get two new regions
        List<Coord> newRegion1, newRegion2;
        Coord region1Tile, region2Tile; // A random tile from each region. We use flooding algorithm to find the rest
        if (splitOnXAxis) {
            region1Tile = officeRegion.Find(c => c.x < wallStartCoord);
            region2Tile = officeRegion.Find(c => c.x > wallStartCoord);
        }
        else {
            region1Tile = officeRegion.Find(c => c.y < wallStartCoord);
            region2Tile = officeRegion.Find(c => c.y > wallStartCoord);
        }

        newRegion1 = GetRegionTiles(region1Tile.x, region1Tile.y, map);
        newRegion2 = GetRegionTiles(region2Tile.x, region2Tile.y, map);

        // Run function recursively
        SplitOfficeRegion(map, newRegion1, config, !splitOnXAxis, false, random);
        SplitOfficeRegion(map, newRegion2, config, !splitOnXAxis, false, random);
    }

    private int[,] AddWallAroundOfficeRegions(int[,] oldMap, OfficeMapConfig config) {
        var mapWithOfficeWalls = oldMap.Clone() as int[,];

        List<List<Coord>> officeRegions = GetRegions(mapWithOfficeWalls, BitMapTypes.ROOM_TYPE);

        foreach (var region in officeRegions) {
            // Get smallest and largest x and y
            var sortedXsAsc = region.Select(c => c.x).ToList().OrderBy(x => x).ToList();
            var sortedYsAsc = region.Select(c => c.y).ToList().OrderBy(y => y).ToList();

            var smallestX = sortedXsAsc[0];
            var biggestX = sortedXsAsc[sortedXsAsc.Count - 1];
            var smallestY = sortedYsAsc[0];
            var biggestY = sortedYsAsc[sortedYsAsc.Count - 1];

            foreach (var coord in region) {
                if (coord.x == smallestX || coord.x == biggestX || coord.y == smallestY || coord.y == biggestY) {
                    mapWithOfficeWalls[coord.x, coord.y] = BitMapTypes.WALL_TYPE;
                }
            }
        }

        return mapWithOfficeWalls;
    }

    private int[,] GenerateMapWithHalls(int[,] map, OfficeMapConfig config, Random random) {
        var mapWithHalls = map.Clone() as int[,];

        // Rotate 90 degrees every time a hallway is generated
        bool usingXAxis = true;
        while (GetHallPercentage(mapWithHalls) < config.maxHallInPercent) {
            // We always split the currently biggest room
            List<List<Coord>> roomRegions = GetRegions(mapWithHalls, BitMapTypes.ROOM_TYPE);
            List<Coord> biggestRoom = roomRegions.OrderByDescending(l => l.Count).ToList()[0];

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
            // Plus 2 to allow for inner walls in the office spaces
            var filteredCoords = sortedCoords.FindAll(x =>
                x + config.minRoomSideLength + 2 < biggestValue - config.minRoomSideLength + 2 && // Right side
                x > smallestValue + config.minRoomSideLength + 2
            );

            if (filteredCoords.Count == 0) {
                Debug.Log("The maxHallInPercent of " + config.maxHallInPercent + " could not be achieved.");
                break;
            }

            // Select random index to fill with hallway
            var selectedIndex = random.Next(filteredCoords.Count);
            var hallStartCoord = filteredCoords[selectedIndex];

            if (usingXAxis) {
                // Fill with hall tiles
                foreach (var c in biggestRoom) {
                    if (hallStartCoord <= c.x && c.x < hallStartCoord + config.hallWidth) {
                        mapWithHalls[c.x, c.y] = BitMapTypes.HALL_TYPE;
                    }
                }

                usingXAxis = false;
            }
            else {
                // Fill with hall tiles
                foreach (var c in biggestRoom) {
                    if (hallStartCoord <= c.y && c.y < hallStartCoord + config.hallWidth) {
                        mapWithHalls[c.x, c.y] = BitMapTypes.HALL_TYPE;
                    }
                }

                usingXAxis = true;
            }
        }

        return mapWithHalls;
    }

    private int[,] GenerateEmptyMap(int width, int height) {
        int[,] emptyMap = new int[width, height];

        for (int x = 0; x < emptyMap.GetLength(0); x++) {
            for (int y = 0; y < emptyMap.GetLength(1); y++) {
                emptyMap[x, y] = BitMapTypes.ROOM_TYPE;
            }
        }

        return emptyMap;
    }

    private float GetHallPercentage(int[,] map) {
        int width = map.GetLength(0);
        int height = map.GetLength(1);
        int mapSize = width * height;

        int amountOfHall = 0;
        for (int x = 0; x < width; x++) {
            for (int y = 0; y < height; y++) {
                amountOfHall += map[x, y] == BitMapTypes.HALL_TYPE ? 1 : 0;
            }
        }

        return (amountOfHall / (float) mapSize) * 100f;
    }

    /**
	 * METHODS for creating cave
	 */
    public SimulationMap<bool> GenerateCaveMap(CaveMapConfig caveConfig, float wallHeight, bool is2D = true) {
        // Clear and destroy objects from previous map
        clearMap();

        var collisionMap = CreateCaveMapWithMesh(caveConfig, wallHeight, is2D);

        ResizePlaneToFitMap(caveConfig.bitMapHeight, caveConfig.bitMapWidth, caveConfig.scaling);

        MovePlaneAndWallRoofToFitWallHeight(wallHeight, is2D);

        return collisionMap;
    }

    private SimulationMap<bool> CreateCaveMapWithMesh(CaveMapConfig caveConfig, float wallHeight = 3.0f,
        bool is2D = true) {
        // Fill map with random walls and empty tiles (Looks kinda like a QR code)
        var randomlyFilledMap = CreateRandomFillMap(caveConfig);

        // Use smoothing runs to make sense of the noise
        // f.x. walls can only stay walls, if they have at least N neighbouring walls
        int[,] smoothedMap = randomlyFilledMap;
        for (int i = 0; i < caveConfig.smoothingRuns; i++) {
            smoothedMap = SmoothMap(smoothedMap, caveConfig);
        }

        // Clean up regions smaller than threshold for both walls and rooms.
        var (survivingRooms, cleanedMap) = RemoveRoomsAndWallsBelowThreshold(caveConfig.wallThresholdSize,
            caveConfig.roomThresholdSize,
            smoothedMap);
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
        var collisionMap = meshGen.GenerateMesh(borderedMap.Clone() as int[,], caveConfig.scaling, wallHeight, is2D,
            false, survivingRooms);

        if (is2D) {
            plane.rotation = Quaternion.AngleAxis(-90, Vector3.right);
        }

        return collisionMap;
    }

    int[,] CreateBorderedMap(int[,] map, int width, int height, int borderSize) {
        int[,] borderedMap = new int[width + (borderSize * 2), height + (borderSize * 2)];

        for (int x = 0; x < borderedMap.GetLength(0); x++) {
            for (int y = 0; y < borderedMap.GetLength(1); y++) {
                if (x >= borderSize && x < width + borderSize && y >= borderSize && y < height + borderSize) {
                    borderedMap[x, y] = map[x - borderSize, y - borderSize];
                }
                else {
                    borderedMap[x, y] = BitMapTypes.WALL_TYPE;
                }
            }
        }

        return borderedMap;
    }

    (List<Room> surviningRooms, int[,] map) RemoveRoomsAndWallsBelowThreshold(int wallThreshold, int roomThreshold,
        int[,] map) {
        var cleanedMap = map.Clone() as int[,];
        List<List<Coord>> wallRegions = GetRegions(cleanedMap, BitMapTypes.WALL_TYPE);

        foreach (List<Coord> wallRegion in wallRegions) {
            if (wallRegion.Count < wallThreshold) {
                foreach (Coord tile in wallRegion) {
                    cleanedMap[tile.x, tile.y] = BitMapTypes.ROOM_TYPE;
                }
            }
        }

        List<List<Coord>> roomRegions = GetRegions(cleanedMap, BitMapTypes.ROOM_TYPE);
        List<Room> survivingRooms = new List<Room>();

        foreach (List<Coord> roomRegion in roomRegions) {
            if (roomRegion.Count < roomThreshold) {
                foreach (Coord tile in roomRegion) {
                    cleanedMap[tile.x, tile.y] = BitMapTypes.WALL_TYPE;
                }
            }
            else {
                survivingRooms.Add(new Room(roomRegion, cleanedMap));
            }
        }

        return (survivingRooms, cleanedMap);
    }

    private int[,] ConnectAllRoomsToMainRoom(List<Room> survivingRooms, int[,] map, CaveMapConfig config) {
        var connectedMap = map.Clone() as int[,];
        survivingRooms.Sort();
        survivingRooms[0].isMainRoom = true;
        survivingRooms[0].isAccessibleFromMainRoom = true;

        return ConnectClosestRooms(survivingRooms, connectedMap, config.connectionPassagesWidth);
    }

    private int[,] ConnectClosestRooms(List<Room> allRooms, int[,] map, int passageWidth) {
        int[,] connectedMap = map.Clone() as int[,];
        List<Room> roomListA = new List<Room>();
        List<Room> roomListB = new List<Room>();


        foreach (Room room in allRooms) {
            if (room.isAccessibleFromMainRoom) {
                roomListB.Add(room);
            }
            else {
                roomListA.Add(room);
            }
        }


        int bestDistance = 0;
        Coord bestTileA = new Coord();
        Coord bestTileB = new Coord();
        Room bestRoomA = new Room();
        Room bestRoomB = new Room();
        bool possibleConnectionFound = false;

        foreach (Room roomA in roomListA) {
            foreach (Room roomB in roomListB) {
                if (roomA == roomB || roomA.IsConnected(roomB)) {
                    continue;
                }

                for (int tileIndexA = 0; tileIndexA < roomA.edgeTiles.Count; tileIndexA++) {
                    for (int tileIndexB = 0; tileIndexB < roomB.edgeTiles.Count; tileIndexB++) {
                        Coord tileA = roomA.edgeTiles[tileIndexA];
                        Coord tileB = roomB.edgeTiles[tileIndexB];
                        int distanceBetweenRooms =
                            (int) (Mathf.Pow(tileA.x - tileB.x, 2) + Mathf.Pow(tileA.y - tileB.y, 2));

                        if (distanceBetweenRooms < bestDistance || !possibleConnectionFound) {
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

        if (possibleConnectionFound) {
            CreatePassage(bestRoomA, bestRoomB, bestTileA, bestTileB, connectedMap, passageWidth);
            connectedMap = ConnectClosestRooms(allRooms, connectedMap, passageWidth);
        }

        return connectedMap;
    }

    void CreatePassage(Room roomA, Room roomB, Coord tileA, Coord tileB, int[,] map, int passageWidth) {
        Room.ConnectRooms(roomA, roomB);
        // Debug.DrawLine (CoordToWorldPoint (tileA), CoordToWorldPoint (tileB), Color.green, 10);

        List<Coord> line = GetLine(tileA, tileB);
        foreach (Coord c in line) {
            MakeRoomOfLine(c, passageWidth, map);
        }
    }

    private void MakeRoomOfLine(Coord c, int r, int[,] map) {
        for (int x = -r; x <= r; x++) {
            for (int y = -r; y <= r; y++) {
                if (x * x + y * y <= r * r) {
                    int drawX = c.x + x;
                    int drawY = c.y + y;
                    if (IsInMapRange(drawX, drawY, map)) {
                        map[drawX, drawY] = BitMapTypes.ROOM_TYPE;
                    }
                }
            }
        }
    }

    private List<Coord> GetLine(Coord from, Coord to) {
        List<Coord> line = new List<Coord>();

        int x = from.x;
        int y = from.y;

        int dx = to.x - from.x;
        int dy = to.y - from.y;

        bool inverted = false;
        int step = Math.Sign(dx);
        int gradientStep = Math.Sign(dy);

        int longest = Mathf.Abs(dx);
        int shortest = Mathf.Abs(dy);

        if (longest < shortest) {
            inverted = true;
            longest = Mathf.Abs(dy);
            shortest = Mathf.Abs(dx);

            step = Math.Sign(dy);
            gradientStep = Math.Sign(dx);
        }

        int gradientAccumulation = longest / 2;
        for (int i = 0; i < longest; i++) {
            line.Add(new Coord(x, y));

            if (inverted) {
                y += step;
            }
            else {
                x += step;
            }

            gradientAccumulation += shortest;
            if (gradientAccumulation >= longest) {
                if (inverted) {
                    x += gradientStep;
                }
                else {
                    y += gradientStep;
                }

                gradientAccumulation -= longest;
            }
        }

        return line;
    }

    // Just used be drawing a line for debugging
    private Vector3 CoordToWorldPoint(Coord tile, int width, int height) {
        return new Vector3(-width / 2 + .5f + tile.x, 2, -height / 2 + .5f + tile.y);
    }

    private List<List<Coord>> GetRegions(int[,] map, params int[] tileTypes) {
        List<List<Coord>> regions = new List<List<Coord>>();
        // Flags if a given coordinate has already been accounted for
        // 1 = yes, 0 = no
        int[,] mapFlags = new int[map.GetLength(0), map.GetLength(1)];
        int counted = 1, notCounted = 0;

        for (int x = 0; x < map.GetLength(0); x++) {
            for (int y = 0; y < map.GetLength(1); y++) {
                if (mapFlags[x, y] == notCounted && tileTypes.Contains(map[x, y])) {
                    List<Coord> newRegion = GetRegionTiles(x, y, map);
                    regions.Add(newRegion);

                    foreach (Coord tile in newRegion) {
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
    private List<Coord> GetRegionTiles(int startX, int startY, int[,] map) {
        List<Coord> tiles = new List<Coord>();
        int[,] mapFlags = new int[map.GetLength(0), map.GetLength(1)];
        int counted = 1, notCounted = 0;
        int tileType = map[startX, startY];

        Queue<Coord> queue = new Queue<Coord>();
        queue.Enqueue(new Coord(startX, startY));
        mapFlags[startX, startY] = counted;

        while (queue.Count > 0) {
            Coord tile = queue.Dequeue();
            tiles.Add(tile);

            for (int x = tile.x - 1; x <= tile.x + 1; x++) {
                for (int y = tile.y - 1; y <= tile.y + 1; y++) {
                    if (IsInMapRange(x, y, map) && (y == tile.y || x == tile.x)) {
                        if (mapFlags[x, y] == notCounted && map[x, y] == tileType) {
                            mapFlags[x, y] = BitMapTypes.WALL_TYPE;
                            queue.Enqueue(new Coord(x, y));
                        }
                    }
                }
            }
        }

        return tiles;
    }

    bool IsInMapRange(int x, int y, int[,] map) {
        return x >= 0 && x < map.GetLength(0) && y >= 0 && y < map.GetLength(1);
    }


    int[,] CreateRandomFillMap(CaveMapConfig config) {
        int[,] randomFillMap = new int[config.bitMapWidth, config.bitMapHeight];
        System.Random pseudoRandom = new System.Random(config.randomSeed);

        for (int x = 0; x < config.bitMapWidth; x++) {
            for (int y = 0; y < config.bitMapHeight; y++) {
                if (x == 0 || x == config.bitMapWidth - 1 || y == 0 || y == config.bitMapHeight - 1) {
                    randomFillMap[x, y] = BitMapTypes.WALL_TYPE;
                }
                else {
                    randomFillMap[x, y] = (pseudoRandom.Next(0, 100) < config.randomFillPercent)
                        ? BitMapTypes.WALL_TYPE
                        : BitMapTypes.ROOM_TYPE;
                }
            }
        }

        return randomFillMap;
    }

    int[,] SmoothMap(int[,] map, CaveMapConfig config) {
        var smoothedMap = map.Clone() as int[,];
        for (int x = 0; x < config.bitMapWidth; x++) {
            for (int y = 0; y < config.bitMapHeight; y++) {
                int neighbourWallTiles = GetSurroundingWallCount(x, y, map);

                if (neighbourWallTiles > config.neighbourWallsNeededToStayWall)
                    smoothedMap[x, y] = BitMapTypes.WALL_TYPE;
                else if (neighbourWallTiles < config.neighbourWallsNeededToStayWall)
                    smoothedMap[x, y] = BitMapTypes.ROOM_TYPE;
            }
        }

        return smoothedMap;
    }

    int GetSurroundingWallCount(int gridX, int gridY, int[,] map) {
        int wallCount = 0;
        for (int neighbourX = gridX - 1; neighbourX <= gridX + 1; neighbourX++) {
            for (int neighbourY = gridY - 1; neighbourY <= gridY + 1; neighbourY++) {
                if (IsInMapRange(neighbourX, neighbourY, map)) {
                    if (neighbourX != gridX || neighbourY != gridY) {
                        wallCount += map[neighbourX, neighbourY];
                    }
                }
                else {
                    wallCount++;
                }
            }
        }

        return wallCount;
    }

    // Draw the gizmo of the map for debugging purposes.
    void drawMap(int[,] map) {
        if (mapToDraw != null) {
            int width = map.GetLength(0);
            int height = map.GetLength(1);

            for (int x = 0; x < width; x++) {
                for (int y = 0; y < height; y++) {
                    switch (map[x, y]) {
                        case BitMapTypes.WALL_TYPE:
                            Gizmos.color = Color.black;
                            break;
                        case BitMapTypes.ROOM_TYPE:
                            Gizmos.color = Color.white;
                            break;
                        case BitMapTypes.HALL_TYPE:
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

    private void OnDrawGizmosSelected() {
        drawMap(mapToDraw);
    }
}