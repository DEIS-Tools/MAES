using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;


namespace Dora.MapGeneration {
	public class Room : IComparable<Room> {
		public List<Coord> tiles;
		// The tiles next to the walls surrounding the room
		public List<Coord> edgeTiles;
		public List<Room> connectedRooms;
		public int roomSize;
		public bool isAccessibleFromMainRoom;
		public bool isMainRoom;
		public bool isHallWay;

		public Room() {
		}

		public Room(List<Coord> roomTiles, int[,] map) {
			tiles = roomTiles;
			roomSize = tiles.Count;
			connectedRooms = new List<Room>();
			
			edgeTiles = new List<Coord>();
			foreach (Coord tile in tiles) {
				for (int x = tile.x - 1; x <= tile.x + 1; x++) {
					for (int y = tile.y - 1; y <= tile.y + 1; y++) {
						if (x == tile.x || y == tile.y) {
							if (map[x,y] == BitMapTypes.WALL_TYPE) {
								edgeTiles.Add(tile);
							}
						}
					}
				}
			}

			this.isHallWay = map[tiles[0].x, tiles[0].y] == BitMapTypes.HALL_TYPE;
		}

		public bool IsWithinRangeOf(Room other, int range) {
			foreach (var tile in this.edgeTiles) {
				foreach (var oTile in other.edgeTiles) {
					if (tile.ManhattanDistanceTo(oTile) <= range)
						return true;
				}
			}

			return false;
		}
		
		public List<Coord> GetWallSurroundingRoom(int wallThickness = 1) {
			var surroundingWalls = new List<Coord>();
			
			int smallestXValue, biggestXValue;
			int smallestYValue, biggestYValue;
			
			// x values in ascending order
			tiles.Sort((c1, c2) => c1.x - c2.x);
			smallestXValue = tiles[0].x;
			biggestXValue = tiles[tiles.Count - 1].x;
			// Sorted by y values in ascending order
			tiles.Sort((c1,c2) => c1.y - c2.y);
			smallestYValue = tiles[0].y;
			biggestYValue = tiles[tiles.Count - 1].y;
			
			for (int x = smallestXValue - wallThickness; x <= biggestXValue + wallThickness; x++) {
				for (int y = smallestYValue - wallThickness; y <= biggestYValue + wallThickness; y++) {
					// If adjacent with a tile, but not in tiles
					var c = new Coord(x, y);
					foreach (var t in this.edgeTiles) {
						// It must not be part of the room, but adjacent to a tile in the room
						if (c.IsAdjacentTo(t) && !this.tiles.Contains(c)) {
							surroundingWalls.Add(c);
						}
					}
				}
			}

			return surroundingWalls;
		}

		public List<Coord> GetSharedWallTiles(Room other, int wallThickness = 1) {
			var walls = this.GetWallSurroundingRoom(wallThickness);
			var otherWalls = other.GetWallSurroundingRoom(wallThickness);

			var intersect = walls.Intersect(otherWalls).ToList();

			foreach (var p in intersect) {
				if (p.x > 59 || p.y > 59) {
					Debug.Log("Too large point at x:" + p.x + ", y:" + p.y);
				}
			}

			return walls.Intersect(otherWalls).ToList();
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
				roomB.SetAccessibleFromMainRoom();
			} else if (roomB.isAccessibleFromMainRoom) {
				roomA.SetAccessibleFromMainRoom();
			}
			roomA.connectedRooms.Add(roomB);
			roomB.connectedRooms.Add(roomA);
		}

		public bool IsConnected(Room otherRoom) {
			return connectedRooms.Contains(otherRoom);
		}

		public int CompareTo(Room otherRoom) {
			return otherRoom.roomSize.CompareTo(roomSize);
		}

		public int RoomSizeExcludingEdgeTiles() {
			return tiles.Count - edgeTiles.Count;
		}

		public void OffsetCoordsBy(int x, int y) {
			var newTiles = new List<Coord>();
			var newEdgeTiles = new List<Coord>();
			
			foreach (var t in tiles) {
				newTiles.Add(new Coord(t.x + x, t.y + y));
			}
			foreach (var t in edgeTiles) {
				newEdgeTiles.Add(new Coord(t.x + x, t.y + y));
			}

			this.tiles = newTiles;
			this.edgeTiles = newEdgeTiles;
		}
	}
}