using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using System;
using System.Linq.Expressions;
using System.Numerics;
using System.Threading;
using Dora.MapGeneration;
using Quaternion = UnityEngine.Quaternion;
using Vector3 = UnityEngine.Vector3;

public class MapGenerator : MonoBehaviour {
	
	private const int WALL_TYPE = 1, ROOM_TYPE = 0;

	public Transform plane;
	public Transform innerWalls;
	public Transform wallRoof;

	public MeshGenerator meshGenerator;

	// Variable used for drawing gizmos on selection for debugging.
	private int[,] mapToDraw = null; 
	
	void Update() {
		if (Input.GetButtonUp("Jump"))
		{
			var config = new CaveMapConfig(100,
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
										true);
		}
		
		

		if (Input.GetMouseButtonDown(1) && (Input.GetKey(KeyCode.LeftControl) || Input.GetKey(KeyCode.RightControl)))
		{
			clearMap();
		}
	}

	public int[,] GenerateOfficeMap(OfficeMapConfig config, float wallHeight, bool is2D = true)
	{
		return null;
	}

	public int[,] GenerateCaveMap(CaveMapConfig caveConfig, float wallHeight, bool is2D = true)
	{
		// Clear and destroy objects from previous map
		clearMap();

		var map = CreateCaveMapWithMesh(caveConfig, wallHeight, is2D);

		ResizePlaneToFitMap(caveConfig.height, caveConfig.width, caveConfig.scaling);

		MovePlaneAndWallRoofToFitWallHeight(wallHeight, is2D);

		return map;
	}

	private void MovePlaneAndWallRoofToFitWallHeight(float wallHeight, bool is2D = true)
	{
		// Move walls and wall roof to above plane depending on wall height
		// The axis depends on whether it is 3D or 2D.
		if (is2D)
		{
			Debug.Log("Correct 2D");
			Vector3 newPosition = wallRoof.position;
			newPosition.z = -wallHeight;
			wallRoof.position = newPosition;
		
			newPosition = innerWalls.position;
			newPosition.z = -wallHeight; 
			innerWalls.position = newPosition;
		}
		else
		{
			Vector3 newPosition = wallRoof.position;
			newPosition.y = wallHeight;
			wallRoof.position = newPosition;
		
			newPosition = innerWalls.position;
			newPosition.y = wallHeight; 
			innerWalls.position = newPosition;
		}
	}

	private void ResizePlaneToFitMap(int height, int width, float scaling, float padding = 0.1f)
	{
		// Resize plane below cave to fit size
		plane.localScale = new Vector3(((width * scaling) / 10f) + padding, 
			1, 
			((height * scaling) / 10f) + padding);
	}

	public void clearMap()
	{
		meshGenerator.ClearMesh();
	}

	private int[,] CreateCaveMapWithMesh(CaveMapConfig caveConfig, float wallHeight = 3.0f, bool is2D = true) {
		// Fill map with random walls and empty tiles (Looks kinda like a QR code)
		var randomlyFilledMap = CreateRandomFillMap(caveConfig);
		
		// Use smoothing runs to make sense of the noise
		// f.x. walls can only stay walls, if they have at least N neighbouring walls
		int[,] smoothedMap = randomlyFilledMap;
		for (int i = 0; i < caveConfig.smoothingRuns; i++)
		{
			smoothedMap = SmoothMap(smoothedMap, caveConfig);
		}

		// Clean up regions smaller than threshold for both walls and rooms.
		var (survivingRooms, cleanedMap) = RemoveRoomsAndWallsBelowThreshold(caveConfig.wallThresholdSize, 
																							caveConfig.roomThresholdSize, 
																							smoothedMap);

		// Connect all rooms to main (the biggest) room
		var connectedMap = ConnectAllRoomsToMainRoom(survivingRooms, cleanedMap, caveConfig);

		// Ensure a border around the map
		var borderedMap = CreateBorderedMap(connectedMap, caveConfig);
		
		// Draw gizmo of map for debugging. Will draw the map in Scene upon selection.
		// mapToDraw = borderedMap;
		
		MeshGenerator meshGen = GetComponent<MeshGenerator>();
		meshGen.GenerateMesh(borderedMap.Clone() as int[,], caveConfig.scaling, wallHeight, is2D);

		if (is2D)
		{
			plane.rotation = Quaternion.AngleAxis(-90, Vector3.right);
		}

		return borderedMap;
	}

	int[,] CreateBorderedMap(int[,] map, CaveMapConfig config)
	{
		int[,] borderedMap = new int[config.width + config.borderSize * 2, config.height + config.borderSize * 2];

		for (int x = 0; x < borderedMap.GetLength(0); x ++) {
			for (int y = 0; y < borderedMap.GetLength(1); y ++) {
				if (x >= config.borderSize && x < config.width + config.borderSize && y >= config.borderSize && y < config.height + config.borderSize) {
					borderedMap[x,y] = map[x - config.borderSize, y - config.borderSize];
				}
				else {
					borderedMap[x,y] = WALL_TYPE;
				}
			}
		}

		return borderedMap;
	}

	(List<Room> surviningRooms, int[,] map) RemoveRoomsAndWallsBelowThreshold(int wallThreshold, int roomThreshold, int[,] map)
	{
		var cleanedMap = map.Clone() as int[,];
		List<List<Coord>> wallRegions = GetRegions (WALL_TYPE, cleanedMap);

		foreach (List<Coord> wallRegion in wallRegions) {
			if (wallRegion.Count < wallThreshold) {
				foreach (Coord tile in wallRegion) {
					cleanedMap[tile.tileX,tile.tileY] = ROOM_TYPE;
				}
			}
		}

		List<List<Coord>> roomRegions = GetRegions (ROOM_TYPE, cleanedMap);
		List<Room> survivingRooms = new List<Room> ();
		
		foreach (List<Coord> roomRegion in roomRegions) {
			if (roomRegion.Count < roomThreshold) {
				foreach (Coord tile in roomRegion) {
					cleanedMap[tile.tileX,tile.tileY] = WALL_TYPE;
				}
			}
			else {
				survivingRooms.Add(new Room(roomRegion, cleanedMap));
			}
		}
		
		return (survivingRooms, cleanedMap);
	}
	
	private int[,] ConnectAllRoomsToMainRoom(List<Room> survivingRooms, int[,] map, CaveMapConfig config)
	{
		var connectedMap = map.Clone() as int[,];
		survivingRooms.Sort ();
		survivingRooms [0].isMainRoom = true;
		survivingRooms [0].isAccessibleFromMainRoom = true;

		return ConnectClosestRooms(survivingRooms, connectedMap, config);
	}

	private int[,] ConnectClosestRooms(List<Room> allRooms, int[,] map, CaveMapConfig config)
	{
		int[,] connectedMap = map.Clone() as int[,];
		List<Room> roomListA = new List<Room> ();
		List<Room> roomListB = new List<Room> ();

		
		foreach (Room room in allRooms) {
			if (room.isAccessibleFromMainRoom) {
				roomListB.Add (room);
			} else {
				roomListA.Add (room);
			}
		}
		

		int bestDistance = 0;
		Coord bestTileA = new Coord ();
		Coord bestTileB = new Coord ();
		Room bestRoomA = new Room ();
		Room bestRoomB = new Room ();
		bool possibleConnectionFound = false;

		foreach (Room roomA in roomListA) {
			foreach (Room roomB in roomListB) {
				if (roomA == roomB || roomA.IsConnected(roomB)) {
					continue;
				}
			
				for (int tileIndexA = 0; tileIndexA < roomA.edgeTiles.Count; tileIndexA ++) {
					for (int tileIndexB = 0; tileIndexB < roomB.edgeTiles.Count; tileIndexB ++) {
						Coord tileA = roomA.edgeTiles[tileIndexA];
						Coord tileB = roomB.edgeTiles[tileIndexB];
						int distanceBetweenRooms = (int)(Mathf.Pow (tileA.tileX-tileB.tileX,2) + Mathf.Pow (tileA.tileY-tileB.tileY,2));

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
			CreatePassage(bestRoomA, bestRoomB, bestTileA, bestTileB, connectedMap, config);
			connectedMap = ConnectClosestRooms(allRooms, connectedMap, config);
		}

		return connectedMap;
	}

	void CreatePassage(Room roomA, Room roomB, Coord tileA, Coord tileB, int[,] map, CaveMapConfig config) {
		Room.ConnectRooms (roomA, roomB);
		// Debug.DrawLine (CoordToWorldPoint (tileA), CoordToWorldPoint (tileB), Color.green, 10);

		List<Coord> line = GetLine(tileA, tileB);
		foreach (Coord c in line) {
			MakeRoomOfLine(c, config.connectionPassagesWidth, map);
		}
	}

	private void MakeRoomOfLine(Coord c, int r, int[,] map) {
		for (int x = -r; x <= r; x++) {
			for (int y = -r; y <= r; y++) {
				if (x*x + y*y <= r*r) {
					int drawX = c.tileX + x;
					int drawY = c.tileY + y;
					if (IsInMapRange(drawX, drawY, map)) {
						map[drawX, drawY] = ROOM_TYPE;
					}
				}
			}
		}
	}

	private List<Coord> GetLine(Coord from, Coord to) {
		List<Coord> line = new List<Coord> ();

		int x = from.tileX;
		int y = from.tileY;

		int dx = to.tileX - from.tileX;
		int dy = to.tileY - from.tileY;

		bool inverted = false;
		int step = Math.Sign (dx);
		int gradientStep = Math.Sign (dy);

		int longest = Mathf.Abs (dx);
		int shortest = Mathf.Abs (dy);

		if (longest < shortest) {
			inverted = true;
			longest = Mathf.Abs(dy);
			shortest = Mathf.Abs(dx);

			step = Math.Sign (dy);
			gradientStep = Math.Sign (dx);
		}

		int gradientAccumulation = longest / 2;
		for (int i =0; i < longest; i ++) {
			line.Add(new Coord(x,y));

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
		return new Vector3 (-width / 2 + .5f + tile.tileX, 2, -height / 2 + .5f + tile.tileY);
	}

	private List<List<Coord>> GetRegions(int tileType, int[,] map) {
		List<List<Coord>> regions = new List<List<Coord>> ();
		// Flags if a given coordinate has already been accounted for
		// 1 = yes, 0 = no
		int[,] mapFlags = new int[map.GetLength(0), map.GetLength(1)];
		int counted = 1, notCounted = 0;

		for (int x = 0; x < map.GetLength(0); x ++) {
			for (int y = 0; y < map.GetLength(1); y ++) {
				if (mapFlags[x,y] == notCounted && map[x,y] == tileType) {
					List<Coord> newRegion = GetRegionTiles(x,y, map);
					regions.Add(newRegion);

					foreach (Coord tile in newRegion) {
						mapFlags[tile.tileX, tile.tileY] = counted;
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
		List<Coord> tiles = new List<Coord> ();
		int[,] mapFlags = new int[map.GetLength(0),  map.GetLength(1)];
		int counted = 1, notCounted = 0;
		int tileType = map [startX, startY];

		Queue<Coord> queue = new Queue<Coord> ();
		queue.Enqueue (new Coord (startX, startY));
		mapFlags [startX, startY] = counted;

		while (queue.Count > 0) {
			Coord tile = queue.Dequeue();
			tiles.Add(tile);

			for (int x = tile.tileX - 1; x <= tile.tileX + 1; x++) {
				for (int y = tile.tileY - 1; y <= tile.tileY + 1; y++) {
					if (IsInMapRange(x,y, map) && (y == tile.tileY || x == tile.tileX)) {
						if (mapFlags[x,y] == notCounted && map[x,y] == tileType) {
							mapFlags[x,y] = WALL_TYPE;
							queue.Enqueue(new Coord(x,y));
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


	int[,] CreateRandomFillMap(CaveMapConfig config)	
	{
		int[,] randomFillMap = new int[config.width, config.height];
		System.Random pseudoRandom = new System.Random(config.randomSeed.GetHashCode());
		
		for (int x = 0; x < config.width; x ++) {
			for (int y = 0; y < config.height; y ++) {
				if (x == 0 || x == config.width - 1 || y == 0 || y == config.height -1) {
					randomFillMap[x,y] = WALL_TYPE;
				}
				else {
					randomFillMap[x,y] = (pseudoRandom.Next(0,100) < config.randomFillPercent) ? WALL_TYPE : ROOM_TYPE;
				}
			}
		}

		return randomFillMap;
	}

	int[,] SmoothMap(int[,] map, CaveMapConfig config) {
		var smoothedMap = map.Clone() as int[,];
		for (int x = 0; x < config.width; x ++) {
			for (int y = 0; y < config.height; y ++) {
				int neighbourWallTiles = GetSurroundingWallCount(x,y, map);

				if (neighbourWallTiles > config.neighbourWallsNeededToStayWall)
					smoothedMap[x,y] = WALL_TYPE;
				else if (neighbourWallTiles < config.neighbourWallsNeededToStayWall)
					smoothedMap[x,y] = ROOM_TYPE;

			}
		}

		return smoothedMap;
	}

	int GetSurroundingWallCount(int gridX, int gridY, int[,] map) {
		int wallCount = 0;
		for (int neighbourX = gridX - 1; neighbourX <= gridX + 1; neighbourX ++) {
			for (int neighbourY = gridY - 1; neighbourY <= gridY + 1; neighbourY ++) {
				if (IsInMapRange(neighbourX, neighbourY, map)) {
					if (neighbourX != gridX || neighbourY != gridY) {
						wallCount += map[neighbourX,neighbourY];
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
	void drawMap(int[,] map)
	{
		int width = map.GetLength(0);
		int height = map.GetLength(1);
		
		if (mapToDraw != null)
		{
			for (int x = 0; x < width; x++)
			{
				for (int y = 0; y < height; y++)
				{
					Gizmos.color = (map[x, y] == 1) ? Color.black : Color.white;
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

	struct Coord {
		public int tileX;
		public int tileY;

		public Coord(int x, int y) {
			tileX = x;
			tileY = y;
		}
	}


	class Room : IComparable<Room> {
		public List<Coord> tiles;
		public List<Coord> edgeTiles;
		public List<Room> connectedRooms;
		public int roomSize;
		public bool isAccessibleFromMainRoom;
		public bool isMainRoom;

		public Room() {
		}

		public Room(List<Coord> roomTiles, int[,] map) {
			tiles = roomTiles;
			roomSize = tiles.Count;
			connectedRooms = new List<Room>();

			edgeTiles = new List<Coord>();
			foreach (Coord tile in tiles) {
				for (int x = tile.tileX - 1; x <= tile.tileX + 1; x++) {
					for (int y = tile.tileY - 1; y <= tile.tileY + 1; y++) {
						if (x == tile.tileX || y == tile.tileY) {
							if (map[x,y] == WALL_TYPE) {
								edgeTiles.Add(tile);
							}
						}
					}
				}
			}
		}

		public void SetAccessibleFromMainRoom() {
			if (!isAccessibleFromMainRoom) {
				isAccessibleFromMainRoom = true;
				foreach (Room connectedRoom in connectedRooms) {
					connectedRoom.SetAccessibleFromMainRoom();
				}
			}
		}

		public static void ConnectRooms(Room roomA, Room roomB) {
			if (roomA.isAccessibleFromMainRoom) {
				roomB.SetAccessibleFromMainRoom ();
			} else if (roomB.isAccessibleFromMainRoom) {
				roomA.SetAccessibleFromMainRoom();
			}
			roomA.connectedRooms.Add (roomB);
			roomB.connectedRooms.Add (roomA);
		}

		public bool IsConnected(Room otherRoom) {
			return connectedRooms.Contains(otherRoom);
		}

		public int CompareTo(Room otherRoom) {
			return otherRoom.roomSize.CompareTo (roomSize);
		}
	}

}