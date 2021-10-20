using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using Dora.Utilities;
using UnityEngine;

namespace Dora.MapGeneration
{
    // A SimulationMap represents a map of square tiles, where each tile is divided into 8 triangles
    // This matches the structure of the map exported by the MapGenerator
    public class SimulationMap<TCell> : IEnumerable<(int, TCell)>
    {
        // The width / height of the map measured in tiles
        public readonly int WidthInTiles, HeightInTiles;

        // The scale of the map in world space
        public readonly float Scale;

        // The scaled offset in world space
        public readonly Vector2 ScaledOffset;

        // These rooms do not include the passages between them.
        // They are used for robot spawning
        public readonly List<Room> rooms;

        // The tiles of the map (each tile containing 8 triangle cells) 
        private readonly SimulationMapTile<TCell>[,] _tiles;

        public SimulationMap(Functional.Factory<TCell> cellFactory, int widthInTiles, int heightInTiles, float scale,
            Vector2 scaledOffset, List<Room> rooms) {
            this.rooms = rooms;
            ScaledOffset = scaledOffset;
            Scale = scale;
            WidthInTiles = widthInTiles;
            HeightInTiles = heightInTiles;
            _tiles = new SimulationMapTile<TCell>[widthInTiles, heightInTiles];
            for (var x = 0; x < widthInTiles; x++)
                for (var y = 0; y < heightInTiles; y++)
                    _tiles[x, y] = new SimulationMapTile<TCell>(cellFactory);
        }

        // Private constructor for a pre-specified set of tiles. This is used in the FMap function
        private SimulationMap(SimulationMapTile<TCell>[,] tiles, float scale, Vector2 scaledOffset)
        {
            ScaledOffset = scaledOffset;
            Scale = scale;
            _tiles = tiles;
            WidthInTiles = tiles.GetLength(0);
            HeightInTiles = tiles.GetLength(1);
        }
        
        public SimulationMapTile<TCell> GetTileByLocalCoordinate(int x, int y)
        {
            return _tiles[x, y];
        }

        // Returns the triangle cell at the given world position
        public TCell GetCell(Vector2 coordinate)
        {
            var localCoordinate = ToLocalMapCoordinate(coordinate);
            var tile = _tiles[(int) localCoordinate.x, (int) localCoordinate.y];
            return tile.GetTriangleCellByCoordinateDecimals(localCoordinate.x % 1.0f, localCoordinate.y % 1.0f);
        }

        // Assigns the given value to the triangle cell at the given coordinate
        public void SetCell(Vector2 coordinate, TCell newCell)
        {
            var localCoordinate = ToLocalMapCoordinate(coordinate);
            var tile = _tiles[(int) localCoordinate.x, (int) localCoordinate.y];
            tile.SetTriangleCellByCoordinateDecimals(localCoordinate.x % 1.0f, localCoordinate.y % 1.0f, newCell);
        }

        // Returns the index of triangle cell at the given world position
        public int GetTriangleIndex(Vector2 coordinate)
        {
            var localCoordinate = ToLocalMapCoordinate(coordinate);
            var tile = _tiles[(int) localCoordinate.x, (int) localCoordinate.y];
            var triangleIndexOffset = ((int) localCoordinate.x) * 8 + ((int) localCoordinate.y) * WidthInTiles * 8;
            return triangleIndexOffset +
                   tile.CoordinateDecimalsToTriangleIndex(localCoordinate.x % 1.0f, localCoordinate.y % 1.0f);
        }

        // Takes a world coordinates and removes the offset and scale to translate it to a local map coordinate
        private Vector2 ToLocalMapCoordinate(Vector2 worldCoordinate)
        {
            var localCoordinate = (worldCoordinate - ScaledOffset) / Scale;
            if (!IsWithinLocalMapBounds(localCoordinate))
            {
                throw new ArgumentException("The given coordinate " + localCoordinate
                                                                    + "(World coordinate:" + worldCoordinate + " )"
                                                                    + " is not within map bounds: {" + WidthInTiles +
                                                                    ", " + HeightInTiles + "}");
            }

            return localCoordinate;
        }

        // Checks that the given coordinate is within the local map bounds
        private bool IsWithinLocalMapBounds(Vector2 localCoordinates)
        {
            return localCoordinates.x >= 0.0f && localCoordinates.x < WidthInTiles
                                              && localCoordinates.y >= 0.0f && localCoordinates.y < HeightInTiles;
        }

        // Generates a new SimulationMap<T2> by mapping the given function over all cells of this map
        public SimulationMap<TNewCell> FMap<TNewCell>(Func<TCell, TNewCell> mapper)
        {
            SimulationMapTile<TNewCell>[,] mappedTiles = new SimulationMapTile<TNewCell>[WidthInTiles, HeightInTiles];
            for (int x = 0; x < WidthInTiles; x++)
                for (int y = 0; y < HeightInTiles; y++)
                    mappedTiles[x, y] = _tiles[x, y].FMap(mapper);

            return new SimulationMap<TNewCell>(mappedTiles, this.Scale, this.ScaledOffset);
        }

        // Enumerates all triangles paired with their index
        public IEnumerator<(int, TCell)> GetEnumerator()
        {
            for (int y = 0; y < HeightInTiles; y++)
            {
                for (int x = 0; x < WidthInTiles; x++)
                {
                    var triangles = _tiles[x, y].GetTriangles();
                    for (int t = 0; t < 8; t++)
                    {
                        yield return ((x * 8 + y * WidthInTiles * 8) + t, triangles[t]);
                    }
                }
            }
        }

        IEnumerator IEnumerable.GetEnumerator()
        {
            return GetEnumerator();
        }
        
    }
}