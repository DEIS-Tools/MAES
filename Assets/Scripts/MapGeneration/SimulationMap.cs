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
        public readonly Vector2 Offset;
        
        private readonly SimulationMapTile<TCell>[,] _tiles;
        private readonly AxialLine2D[,,] _tileEdges;
        
        private enum TileEdgeSide
        {
            North, South, East, West
        }
        
        public SimulationMap(Functional.Factory<TCell> cellFactory, int widthInTiles, int heightInTiles, float scale, Vector2 offset)
        {
            this.Offset = offset;
            this.Scale = scale;
            this.WidthInTiles = widthInTiles;
            this.HeightInTiles = heightInTiles;
            _tiles = new SimulationMapTile<TCell>[widthInTiles, heightInTiles];
            _tileEdges = new AxialLine2D[widthInTiles, heightInTiles, 4];
            var tileSize = scale; 
            for (int x = 0; x < widthInTiles; x++)
            {
                for (int y = 0; y < heightInTiles; y++)
                {
                    _tiles[x,y] = new SimulationMapTile<TCell>(cellFactory);
                    var v1 = new Vector2(x, y);
                    var v2 = new Vector2(x + tileSize, y);
                    var v3 = new Vector2(x + tileSize, y + tileSize);
                    var v4 = new Vector2(x, y + tileSize);
                    _tileEdges[x, y, (int) TileEdgeSide.South] = new AxialLine2D(v1, v2);
                    _tileEdges[x, y, (int) TileEdgeSide.East] = new AxialLine2D(v2, v3);
                    _tileEdges[x, y, (int) TileEdgeSide.North] = new AxialLine2D(v4, v3);
                    _tileEdges[x, y, (int) TileEdgeSide.West] = new AxialLine2D(v1, v4);
                }
            }
        }

        // Casts a trace starting at given point, moving in the given direction and terminating when encountering a
        // collision or after exceeding the given direction.
        // Returns the list of intersected cells in the order that they were encountered  
        public List<TCell> Raytrace(Vector2 startingPoint, float angle, float distance)
        {
            // First find the Tiles that intersect with the trace
            
            return new List<TCell>();
        }
        
        private List<SimulationMapTile<TCell>> TilesTrace(Vector2 startingPoint, float angleDegrees, float distance)
        {
            if (Mathf.Abs(angleDegrees) < 0.01f) 
                throw new ArgumentException("Cannot perform raytrace for angles of 90° and 270°");
            
            // Convert angle to a
            // solve for b
            var a = Mathf.Tan(Mathf.PI / 180 * angleDegrees);
            var b = startingPoint.y - a * startingPoint.x;
            
            List<SimulationMapTile<TCell>> tiles = new List<SimulationMapTile<TCell>>();
            tiles.Add(_tiles[(int) startingPoint.x, (int) startingPoint.y]);

            do
            {
                
            } while (true);
        }

        
        // Finds and returns the coordinate of the next tile (ints) and the coordinate of intersection (floats)
        public (Vector2Int, Vector2) FindNextTileIntersection(Vector2Int currentTile, float direction, float a, float b)
        {
            if (direction < 0f || direction > 360f) 
                throw new ArgumentException("Direction must be a negative float between 0 and 360");

            
            
            if (direction < 90f)
            {
                // 
                return (currentTile + Vector2Int.up, currentTile + Vector2Int.right);
            }

            if (direction < 180f)
            {
                return (currentTile + Vector2Int.up, currentTile + Vector2Int.left);                
            }

            if (direction < 270f)
            {
                return (currentTile + Vector2Int.down, currentTile + Vector2Int.left);
            }

            else
            {
                return (currentTile + Vector2Int.down, currentTile + Vector2Int.right);
            }
                
        }

        public int TriangleCount()
        {
            return WidthInTiles * HeightInTiles * 8;
        }

        // Private constructor for a pre-specified set of tiles. This is used in the FMap function
        private SimulationMap(SimulationMapTile<TCell>[,] tiles, float scale, Vector2 offset)
        {
            this.Offset = offset;
            _tiles = tiles;
            WidthInTiles = tiles.GetLength(0);
            HeightInTiles = tiles.GetLength(1);
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
                                            + " is not within map bounds: {" + WidthInTiles + ", " + HeightInTiles + "}");
            }
            return localCoordinate;
        }
        
        // Checks that the given coordinates are within the local map bounds
        private bool IsWithinLocalMapBounds(Vector2 localCoordinates)
        {
            return localCoordinates.x >= 0.0f && localCoordinates.x < WidthInTiles
                   && localCoordinates.y >= 0f && localCoordinates.y < HeightInTiles; 
        }

        // Generates a new SimulationMap<T2> by mapping the given function over all cells
        public SimulationMap<TNewCell> FMap<TNewCell>(Func<TCell, TNewCell> mapper)
        {
            SimulationMapTile<TNewCell>[,] mappedTiles = new SimulationMapTile<TNewCell>[WidthInTiles, HeightInTiles];
            for (int x = 0; x < WidthInTiles; x++)
            {
                for (int y = 0; y < HeightInTiles; y++)
                {
                    mappedTiles[x, y] = _tiles[x, y].FMap(mapper);
                }
            }
            return new SimulationMap<TNewCell>(mappedTiles, this.Scale, this.Offset);
        }

        // Enumerates all triangles and their index
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

        public SimulationMapTile<TCell> GetTile(int x, int y)
        {
            return _tiles[x, y];
        }
    }
}