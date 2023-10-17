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

using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using static Maes.Utilities.Geometry;
using static Maes.Map.MapGen.TileTypes;

namespace Maes.Map.MapGen {
    public class Room : IComparable<Room> {
        public List<Vector2Int> tiles;

        public bool[,] tilesAsArray; // If true, it is contained in room
        // The tiles next to the walls surrounding the room
        public List<Vector2Int> edgeTiles;
        public List<Room> connectedRooms;
        public int roomSize;
        public bool isAccessibleFromMainRoom;
        public bool isMainRoom;
        public bool isHallWay;

        public Room() {
        }

        public Room(List<Vector2Int> roomTiles, int[,] map) {
            tiles = roomTiles;
            roomSize = tiles.Count;
            connectedRooms = new List<Room>();
            tilesAsArray = new bool[map.GetLength(0), map.GetLength(1)];

            edgeTiles = new List<Vector2Int>();
            foreach (Vector2Int tile in tiles) {
                tilesAsArray[tile.x, tile.y] = true;
                for (int x = tile.x - 1; x <= tile.x + 1; x++) {
                    for (int y = tile.y - 1; y <= tile.y + 1; y++) {
                        if ((x == tile.x || y == tile.y) && IsInMapRange(x,y,map)) {
                            if (map[x, y] == WALL_TYPE) {
                                edgeTiles.Add(tile);
                            }
                        }
                    }
                }
            }

            this.isHallWay = map[tiles[0].x, tiles[0].y] == HALL_TYPE;
        }

        private bool IsInMapRange(int x, int y, int[,] map)
        {
            return x >= 0 && x < map.GetLength(0) && y >= 0 && y < map.GetLength(1);
        }

        public bool IsWithinRangeOf(Room other, int range) {
            foreach (var tile in this.edgeTiles) {
                foreach (var oTile in other.edgeTiles) {
                    if(ManhattanDistance(tile, oTile) <= range)
                        return true;
                }
            }

            return false;
        }

        public List<Vector2Int> GetWallSurroundingRoom(int wallThickness = 1) {
            var surroundingWalls = new HashSet<Vector2Int>();

            int smallestXValue, biggestXValue;
            int smallestYValue, biggestYValue;
            
            /*smallestXValue = tiles.Aggregate((agg, next) => next.x < agg.x ? next : agg).x;
            biggestXValue = tiles.Aggregate((agg, next) => next.x > agg.x ? next : agg).x;
            smallestYValue = tiles.Aggregate((agg, next) => next.y < agg.y ? next : agg).y;
            biggestYValue = tiles.Aggregate((agg, next) => next.y > agg.y ? next : agg).y;*/
            smallestXValue = tiles.Min(e => e.x);
            biggestXValue = tiles.Max(e => e.x);
            smallestYValue = tiles.Min(e => e.y);
            biggestYValue = tiles.Max(e => e.y);

            for (int x = smallestXValue - wallThickness; x <= biggestXValue + wallThickness; x++) {
                for (int y = smallestYValue - wallThickness; y <= biggestYValue + wallThickness; y++) {
                    // Check if neighbours are within tiles && this is not in tiles
                    var c = new Vector2Int(x, y);
                    bool isSurroundingWall = false;
                    // If any neighbor is within our tiles, we are part of the surrounding wall
                    for (int xn = x - wallThickness; xn <= x + wallThickness; xn++) {
                        for (int yn = y - wallThickness; yn <= y + wallThickness; yn++) {
                            // Is within bounds of map
                            if ((0 <= xn && xn < tilesAsArray.GetLength(0)) && (0 <= yn && yn < tilesAsArray.GetLength(1))) {
                                if (tilesAsArray[xn, yn] == true) // if contained in tiles
                                    isSurroundingWall = true;
                            }
                        }
                    }
                    if(isSurroundingWall)
                        surroundingWalls.Add(c);

                    // If it is between the edge tiles, i.e. in the middle, continue!
                    /*if ((smallestXValue <= x && x <= biggestXValue) && (smallestYValue <= y && y <= biggestYValue))
                        continue;
                    
                    c = new Coord(x, y);
                    surroundingWalls.Add(c);*/
                }
            }

            /*for (int x = smallestXValue - wallThickness; x <= biggestXValue + wallThickness; x++) {
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
            }*/

            return surroundingWalls.ToList();
        }

        public List<Vector2Int> GetSharedWallTiles(Room other, int wallThickness = 1) {
            var walls = this.GetWallSurroundingRoom(wallThickness);
            var otherWalls = other.GetWallSurroundingRoom(wallThickness);

            var intersect = walls.Intersect(otherWalls).ToList();

            return intersect;
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
            }
            else if (roomB.isAccessibleFromMainRoom) {
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
            var newTiles = new List<Vector2Int>();
            var newEdgeTiles = new List<Vector2Int>();

            foreach (var t in tiles) {
                newTiles.Add(new Vector2Int(t.x + x, t.y + y));
            }

            foreach (var t in edgeTiles) {
                newEdgeTiles.Add(new Vector2Int(t.x + x, t.y + y));
            }

            this.tiles = newTiles;
            this.edgeTiles = newEdgeTiles;
        }
    }
}