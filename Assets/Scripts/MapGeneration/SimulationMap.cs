using System;
using System.Collections.Generic;
using System.Linq;
using Dora.Utilities;
using UnityEngine;

namespace Dora.MapGeneration
{
    // A SimulationMap represents a map of square tiles, where each tile is divided into 8 triangles
    // This matches the structure of the map exported by the MapGenerator
    public class SimulationMap<TCell>
    {
        // The width / height of the map measured in tiles
        public readonly int Width, Height;
        
        // The scale of the map in world space
        public readonly float Scale;
        // The scaled offset in world space
        public readonly Vector2 Offset;
        
        private readonly SimulationMapTile<TCell>[,] _tiles;
        
        public SimulationMap(Functional.Factory<TCell> cellFactory, int width, int height, float scale, Vector2 offset)
        {
            this.Offset = offset;
            this.Scale = scale;
            this.Width = width;
            this.Height = height;
            _tiles = new SimulationMapTile<TCell>[width, height];
            for (int x = 0; x < width; x++)
            {
                for (int y = 0; y < height; y++)
                {
                    _tiles[x,y] = new SimulationMapTile<TCell>(cellFactory);
                }
            }
        }

        // Private constructor for a pre-specified set of tiles. This is used in the fmap function
        private SimulationMap(SimulationMapTile<TCell>[,] tiles, float scale, Vector2 offset)
        {
            this.Offset = offset;
            _tiles = tiles;
            Width = tiles.GetLength(0);
            Height = tiles.GetLength(1);
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

        // Takes a world coordinates and removes the offset and scale to translate it to a local map coordinate
        private Vector2 ToLocalMapCoordinate(Vector2 worldCoordinate)
        {
            var localCoordinate = (worldCoordinate - Offset) / Scale;
            if (!IsWithinLocalMapBounds(localCoordinate))
            {
                throw new ArgumentException("The given coordinate " + localCoordinate 
                                            + "(World coordinate:" + worldCoordinate + " )" 
                                            + " is not within map bounds: {" + Width + ", " + Height + "}");
            }
            return localCoordinate;
        }
        
        // Checks that the given coordinates are within the local map bounds
        private bool IsWithinLocalMapBounds(Vector2 localCoordinates)
        {
            return localCoordinates.x >= 0.0f && localCoordinates.x < Width
                   && localCoordinates.y >= 0f && localCoordinates.y < Height; 
        }

        // Generates a new SimulationMap<T2> by mapping the given function over all cells
        public SimulationMap<TNewCell> FMap<TNewCell>(Func<TCell, TNewCell> mapper)
        {
            SimulationMapTile<TNewCell>[,] mappedTiles = new SimulationMapTile<TNewCell>[Width, Height];
            for (int x = 0; x < Width; x++)
            {
                for (int y = 0; y < Height; y++)
                {
                    mappedTiles[x, y] = _tiles[x, y].FMap(mapper);
                }
            }
            return new SimulationMap<TNewCell>(mappedTiles, this.Scale, this.Offset);
        }
    }
}