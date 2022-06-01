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
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using Maes.Map.MapGen;
using Maes.Utilities;
using UnityEngine;

namespace Maes.Map {
    // A SimulationMap represents a map of square tiles, where each tile is divided into 8 triangles
    // This matches the structure of the map exported by the MapGenerator
    public class SimulationMap<TCell> : IEnumerable<(int, TCell)> {
        // The width / height of the map measured in tiles
        public readonly int WidthInTiles, HeightInTiles;
        
        // The offset in world space
        public readonly Vector2 ScaledOffset;

        // These rooms do not include the passages between them.
        // They are used for robot spawning
        public readonly List<Room> rooms;

        // The tiles of the map (each tile containing 8 triangle cells) 
        private readonly SimulationMapTile<TCell>[,] _tiles;

        public SimulationMap(Functional.Factory<TCell> cellFactory, int widthInTiles, int heightInTiles,
            Vector2 scaledOffset, List<Room> rooms) {
            this.rooms = rooms;
            ScaledOffset = scaledOffset;
            WidthInTiles = widthInTiles;
            HeightInTiles = heightInTiles;
            _tiles = new SimulationMapTile<TCell>[widthInTiles, heightInTiles];
            for (var x = 0; x < widthInTiles; x++)
            for (var y = 0; y < heightInTiles; y++)
                _tiles[x, y] = new SimulationMapTile<TCell>(cellFactory);
        }

        // Private constructor for a pre-specified set of tiles. This is used in the FMap function
        public SimulationMap(SimulationMapTile<TCell>[,] tiles, Vector2 scaledOffset) {
            ScaledOffset = scaledOffset;
            _tiles = tiles;
            WidthInTiles = tiles.GetLength(0);
            HeightInTiles = tiles.GetLength(1);
        }

        public SimulationMapTile<TCell> GetTileByLocalCoordinate(int x, int y) {
            return _tiles[x, y];
        }

        public ((int, TCell), (int, TCell)) GetMiniTilesByCoarseTileCoordinate(Vector2 localCoordinate) {
            var tile = _tiles[(int) localCoordinate.x, (int) localCoordinate.y];
            var triangles = tile.GetTriangles();
            var mainTriangleIndex = tile.CoordinateDecimalsToTriangleIndex(localCoordinate.x % 1.0f, localCoordinate.y % 1.0f);
            // Calculate the number of triangles preceding this tile
            int triangleOffset = ((int) localCoordinate.x) * 8 + ((int) localCoordinate.y) * WidthInTiles * 8;
            if (mainTriangleIndex % 2 == 0)
                return ((mainTriangleIndex + triangleOffset, triangles[mainTriangleIndex]), (mainTriangleIndex + 1 + triangleOffset, triangles[mainTriangleIndex + 1]));
            else
                return ((mainTriangleIndex - 1 + triangleOffset, triangles[mainTriangleIndex - 1]), (mainTriangleIndex + triangleOffset, triangles[mainTriangleIndex]));
        }
        
        // Returns the cells of the tile at the given coordinate along with index of the first cell
        private (int, List<TCell>) GetTileCellsByWorldCoordinate(Vector2 worldCoord) {
            var localCoord = WorldCoordinateToCoarseTileCoordinate(worldCoord);
            int triangleOffset = ((int) localCoord.x) * 8 + ((int) localCoord.y) * WidthInTiles * 8;
            return (triangleOffset, _tiles[(int) localCoord.x, (int) localCoord.y].GetTriangles());
        }
        
        
        /// <param name="worldCoordinate">A coordinate that is within the bounds of the mini-tile in world space</param>
        /// <returns> the pair of triangles that make up the 'mini-tile' at the given world location</returns>
        internal ((int, TCell), (int, TCell)) GetMiniTileTrianglesByWorldCoordinates(Vector2 worldCoordinate) {
            var localCoordinate = WorldCoordinateToCoarseTileCoordinate(worldCoordinate);
            return GetMiniTilesByCoarseTileCoordinate(localCoordinate);
        }

        // Returns the triangle cell at the given world position
        private TCell GetCell(Vector2 coordinate) {
            var localCoordinate = WorldCoordinateToCoarseTileCoordinate(coordinate);
            var tile = _tiles[(int) localCoordinate.x, (int) localCoordinate.y];
            return tile.GetTriangleCellByCoordinateDecimals(localCoordinate.x % 1.0f, localCoordinate.y % 1.0f);
        }

        // Assigns the given value to the triangle cell at the given coordinate
        void SetCell(Vector2 coordinate, TCell newCell) {
            var localCoordinate = WorldCoordinateToCoarseTileCoordinate(coordinate);
            var tile = _tiles[(int) localCoordinate.x, (int) localCoordinate.y];
            tile.SetTriangleCellByCoordinateDecimals(localCoordinate.x % 1.0f, localCoordinate.y % 1.0f, newCell);
        }

        // Returns the index of triangle cell at the given world position
        internal int GetTriangleIndex(Vector2 coordinate) {
            var localCoordinate = WorldCoordinateToCoarseTileCoordinate(coordinate);
            var tile = _tiles[(int) localCoordinate.x, (int) localCoordinate.y];
            var triangleIndexOffset = ((int) localCoordinate.x) * 8 + ((int) localCoordinate.y) * WidthInTiles * 8;
            return triangleIndexOffset +
                   tile.CoordinateDecimalsToTriangleIndex(localCoordinate.x % 1.0f, localCoordinate.y % 1.0f);
        }

        // Takes a world coordinates and removes the offset and scale to translate it to a local map coordinate
        Vector2 WorldCoordinateToCoarseTileCoordinate(Vector2 worldCoordinate) {
            var localCoordinate = (worldCoordinate - ScaledOffset);
            if (!IsWithinLocalMapBounds(localCoordinate)) {
                throw new ArgumentException("The given coordinate " + localCoordinate
                                                                    + "(World coordinate:" + worldCoordinate + " )"
                                                                    + " is not within map bounds: {" + WidthInTiles +
                                                                    ", " + HeightInTiles + "}");
            }

            return localCoordinate;
        }
        
        // Takes a world coordinates and removes the offset and scale to translate it to a local map coordinate
        // This unsafe version may return a coordinate that is outside the bounds of the map
        Vector2 WorldCoordinateToLocalMapCoordinateUnsafe(Vector2 worldCoordinate) {
            return worldCoordinate - ScaledOffset;
        }

        // Checks that the given coordinate is within the local map bounds
        bool IsWithinLocalMapBounds(Vector2 localCoordinates) {
            return localCoordinates.x >= 0.0f && localCoordinates.x < WidthInTiles
                                              && localCoordinates.y >= 0.0f && localCoordinates.y < HeightInTiles;
        }

        // Generates a new SimulationMap<T2> by mapping the given function over all cells of this map
        public SimulationMap<TNewCell> FMap<TNewCell>(Func<TCell, TNewCell> mapper) {
            SimulationMapTile<TNewCell>[,] mappedTiles = new SimulationMapTile<TNewCell>[WidthInTiles, HeightInTiles];
            for (int x = 0; x < WidthInTiles; x++)
            for (int y = 0; y < HeightInTiles; y++)
                mappedTiles[x, y] = _tiles[x, y].FMap(mapper);

            return new SimulationMap<TNewCell>(mappedTiles, this.ScaledOffset);
        }

        // Enumerates all triangles paired with their index
        public IEnumerator<(int, TCell)> GetEnumerator() {
            for (int y = 0; y < HeightInTiles; y++) {
                for (int x = 0; x < WidthInTiles; x++) {
                    var triangles = _tiles[x, y].GetTriangles();
                    for (int t = 0; t < 8; t++) {
                        yield return ((x * 8 + y * WidthInTiles * 8) + t, triangles[t]);
                    }
                }
            }
        }

        IEnumerator IEnumerable.GetEnumerator() {
            return GetEnumerator();
        }
    }
}