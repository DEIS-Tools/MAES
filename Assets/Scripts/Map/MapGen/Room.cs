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

        public Room(List<Vector2Int> roomTiles, Tile[,] map) {
            tiles = roomTiles;
            roomSize = tiles.Count;
            connectedRooms = new List<Room>();
            tilesAsArray = new bool[map.GetLength(0), map.GetLength(1)];

            edgeTiles = new List<Vector2Int>();
            foreach (var tile in tiles) {
                tilesAsArray[tile.x, tile.y] = true;
                for (var x = tile.x - 1; x <= tile.x + 1; x++) {
                    for (var y = tile.y - 1; y <= tile.y + 1; y++)
                    {
                        if ((x != tile.x && y != tile.y) || !IsInMapRange(x, y, map)) 
                            continue;
                        if (Tile.IsWall(map[x, y].Type))
                            edgeTiles.Add(tile);
                    }
                }
            }

            isHallWay = map[tiles[0].x, tiles[0].y].Type == TileType.Hall;
        }

        private static bool IsInMapRange(int x, int y, Tile[,] map)
        {
            return x >= 0 && x < map.GetLength(0) && y >= 0 && y < map.GetLength(1);
        }

        public bool IsWithinRangeOf(Room other, int range)
        {
            return edgeTiles.Any(tile => other.edgeTiles.Any(oTile => ManhattanDistance(tile, oTile) <= range));
        }

        public List<Vector2Int> GetWallSurroundingRoom(int wallThickness = 1) {
            var surroundingWalls = new HashSet<Vector2Int>();

            /*smallestXValue = tiles.Aggregate((agg, next) => next.x < agg.x ? next : agg).x;
            biggestXValue = tiles.Aggregate((agg, next) => next.x > agg.x ? next : agg).x;
            smallestYValue = tiles.Aggregate((agg, next) => next.y < agg.y ? next : agg).y;
            biggestYValue = tiles.Aggregate((agg, next) => next.y > agg.y ? next : agg).y;*/
            var smallestXValue = tiles.Min(e => e.x);
            var biggestXValue = tiles.Max(e => e.x);
            var smallestYValue = tiles.Min(e => e.y);
            var biggestYValue = tiles.Max(e => e.y);

            for (var x = smallestXValue - wallThickness; x <= biggestXValue + wallThickness; x++)
            {
                bool isSurroundingWall = false;
                for (var y = smallestYValue - wallThickness; y <= biggestYValue + wallThickness; y++) {
                    // Check if neighbours are within tiles && this is not in tiles
                    var c = new Vector2Int(x, y);
                    // If any neighbor is within our tiles, we are part of the surrounding wall
                    for (var xn = x - wallThickness; xn <= x + wallThickness; xn++) {
                        for (var yn = y - wallThickness; yn <= y + wallThickness; yn++)
                        {
                            // If out of bounds then skip 
                            if ((0 > xn || xn >= tilesAsArray.GetLength(0)) ||
                                (0 > yn || yn >= tilesAsArray.GetLength(1))) 
                                continue;
                            if (tilesAsArray[xn, yn]) // if contained in tiles
                                isSurroundingWall = true;
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
            var walls = GetWallSurroundingRoom(wallThickness);
            var otherWalls = other.GetWallSurroundingRoom(wallThickness);

            var intersect = walls.Intersect(otherWalls).ToList();

            return intersect;
        }

        public void SetAccessibleFromMainRoom()
        {
            if (isAccessibleFromMainRoom) 
                return;
            isAccessibleFromMainRoom = true;
            foreach (var connectedRoom in connectedRooms) {
                connectedRoom.SetAccessibleFromMainRoom();
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
            var newTiles = tiles.Select(t => new Vector2Int(t.x + x, t.y + y)).ToList();
            var newEdgeTiles = edgeTiles.Select(t => new Vector2Int(t.x + x, t.y + y)).ToList();

            tiles = newTiles;
            edgeTiles = newEdgeTiles;
        }
    }
}