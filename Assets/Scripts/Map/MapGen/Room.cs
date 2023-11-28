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

namespace Maes.Map.MapGen
{
    public class Room : IComparable<Room>
    {
        public List<Vector2Int> Tiles { get; set; }

        public bool[,] TilesAsArray { get; set; } // If true, it is contained in room
        // The tiles next to the walls surrounding the room
        public List<Vector2Int> EdgeTiles { get; set; }
        public List<Room> ConnectedRooms { get; set; }
        public int RoomSize { get; set; }
        public bool IsAccessibleFromMainRoom { get; set; }
        public bool IsMainRoom { get; set; }
        public bool IsHallWay { get; set; }

        public Room()
        {
        }

        public Room(List<Vector2Int> roomTiles, Tile[,] map)
        {
            Tiles = roomTiles;
            RoomSize = Tiles.Count;
            ConnectedRooms = new List<Room>();
            TilesAsArray = new bool[map.GetLength(0), map.GetLength(1)];

            EdgeTiles = new List<Vector2Int>();
            foreach (var tile in Tiles)
            {
                TilesAsArray[tile.x, tile.y] = true;
                for (var x = tile.x - 1; x <= tile.x + 1; x++)
                {
                    for (var y = tile.y - 1; y <= tile.y + 1; y++)
                    {
                        if ((x != tile.x && y != tile.y) || !IsInMapRange(x, y, map))
                            continue;
                        if (Tile.IsWall(map[x, y].Type))
                            EdgeTiles.Add(tile);
                    }
                }
            }

            EdgeTiles = EdgeTiles.Distinct().ToList();
            IsHallWay = map[Tiles[0].x, Tiles[0].y].Type == TileType.Hall;
        }

        private static bool IsInMapRange<T>(int x, int y, T[,] map)
        {
            return x >= 0 && x < map.GetLength(0) && y >= 0 && y < map.GetLength(1);
        }

        public bool IsWithinRangeOf(Room other, int range)
        {
            return EdgeTiles.Any(tile => other.EdgeTiles.Any(oTile => ManhattanDistance(tile, oTile) <= range));
        }

        public List<Vector2Int> GetWallSurroundingRoom(int wallThickness = 1)
        {
            var outsideTiles = new List<Vector2Int>();
            // Add tiles surrounding the room, going a tile further with each step
            //todo rooms are not necessarily square meaning we have to go through every edge/tile not just the outer corners
            foreach (var tile in Tiles)
                for (var x = -wallThickness; x < wallThickness; x++)
                    for (var y = -wallThickness; y < wallThickness; y++)
                        outsideTiles.Add(new Vector2Int(tile.x+x, tile.y+y));

            return outsideTiles.Where(vec => IsInMapRange(vec.x, vec.y, TilesAsArray)).Distinct().ToList();
        }

        public List<Vector2Int> GetSharedWallTiles(Room other, int wallThickness = 1)
        {
            var walls = GetWallSurroundingRoom(wallThickness);
            var otherWalls = other.GetWallSurroundingRoom(wallThickness);

            var intersect = walls.Intersect(otherWalls).ToList();

            return intersect;
        }

        public void SetAccessibleFromMainRoom()
        {
            if (IsAccessibleFromMainRoom)
                return;
            IsAccessibleFromMainRoom = true;
            foreach (var connectedRoom in ConnectedRooms)
            {
                connectedRoom.SetAccessibleFromMainRoom();
            }
        }

        public static void ConnectRooms(Room roomA, Room roomB)
        {
            if (roomA.IsAccessibleFromMainRoom)
            {
                roomB.SetAccessibleFromMainRoom();
            }
            else if (roomB.IsAccessibleFromMainRoom)
            {
                roomA.SetAccessibleFromMainRoom();
            }

            roomA.ConnectedRooms.Add(roomB);
            roomB.ConnectedRooms.Add(roomA);
        }

        public bool IsConnected(Room otherRoom)
        {
            return ConnectedRooms.Contains(otherRoom);
        }

        public int CompareTo(Room otherRoom)
        {
            return otherRoom.RoomSize.CompareTo(RoomSize);
        }

        public int RoomSizeExcludingEdgeTiles()
        {
            return Tiles.Count - EdgeTiles.Count;
        }

        public void OffsetCoordsBy(int x, int y)
        {
            var newTiles = Tiles.Select(t => new Vector2Int(t.x + x, t.y + y)).ToList();
            var newEdgeTiles = EdgeTiles.Select(t => new Vector2Int(t.x + x, t.y + y)).ToList();

            Tiles = newTiles;
            EdgeTiles = newEdgeTiles;
        }
    }
}