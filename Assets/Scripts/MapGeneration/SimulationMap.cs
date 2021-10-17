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
        private AxialLine2D[,,] _tileEdges;
        
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
            for (int x = 0; x < widthInTiles; x++)
            {
                for (int y = 0; y < heightInTiles; y++)
                {
                    _tiles[x, y] = new SimulationMapTile<TCell>(cellFactory);
                }
            }
            GenerateTileEdges(widthInTiles, heightInTiles, scale);
        }
        
        // Private constructor for a pre-specified set of tiles. This is used in the FMap function
        private SimulationMap(SimulationMapTile<TCell>[,] tiles, float scale, Vector2 offset)
        {
            this.Offset = offset;
            _tiles = tiles;
            this.Scale = scale;
            WidthInTiles = tiles.GetLength(0);
            HeightInTiles = tiles.GetLength(1);
            GenerateTileEdges(WidthInTiles, HeightInTiles, scale);
        }

        private void GenerateTileEdges(int widthInTiles, int heightInTiles, float tileSize)
        {
            _tileEdges = new AxialLine2D[widthInTiles, heightInTiles, 4];
            for (int x = 0; x < widthInTiles; x++)
            {
                for (int y = 0; y < heightInTiles; y++)
                {
                    var v1 = new Vector2(x, y) * Scale + Offset;
                    var v2 = new Vector2(x + 1, y)  * Scale + Offset;
                    var v3 = new Vector2(x + 1, y + 1)  * Scale + Offset;
                    var v4 = new Vector2(x, y + 1)  * Scale + Offset;
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
        public List<TCell> Raytrace(Vector2 startingPoint, float angle, float distance, Action<TCell> cellAction)
        {
            // First find the Tiles that intersect with the trace
            var tracedTiles = TilesTrace(startingPoint, angle, distance);
            
            var tracedCells = new List<TCell>();
            foreach (var tile in tracedTiles)
            {
                tile.ForEachCell(cellAction);
            }
            return tracedCells;
        }
        
        private List<SimulationMapTile<TCell>> TilesTrace(Vector2 startingPoint, float angleDegrees, float rayLength)
        {
            if (Mathf.Abs(angleDegrees) < 0.01f) 
                throw new ArgumentException("Cannot perform raytrace for angles of 90° and 270°");
            
            // Convert angle to a
            // solve for b
            var a = Mathf.Tan(Mathf.PI / 180 * angleDegrees);
            var b = startingPoint.y - a * startingPoint.x;
            
            List<SimulationMapTile<TCell>> tracedTiles = new List<SimulationMapTile<TCell>>();
            var localCoordinate = (startingPoint - Offset) / Scale;
            var currentTile = new Vector2Int((int) localCoordinate.x, (int) localCoordinate.y);
            Vector2 intersection;
            do
            {
                tracedTiles.Add(_tiles[currentTile.x, currentTile.y]);
                Vector2Int nextTile;
                (nextTile, intersection) = FindNextTileIntersection(currentTile, angleDegrees, a, b);
                if (!IsWithinLocalMapBounds(nextTile)) break;
                currentTile = nextTile;
            } while (Geometry.DistanceBetween(startingPoint, intersection) <= rayLength); 

            return tracedTiles;
        }

        
        // Finds and returns the coordinate of the next tile (ints) and the coordinate of intersection (floats)
        public (Vector2Int, Vector2) FindNextTileIntersection(Vector2Int localTileCoordinate, float direction, float a, float b)
        {
            if (direction < 0f || direction > 360f) 
                throw new ArgumentException("Direction must be a negative float between 0 and 360");
            
            
            if (direction < 90f)
            {
                // The line must either exit the North edge or East edge
                // Check north first
                var northIntersection = GetTileEdge(localTileCoordinate, TileEdgeSide.North).GetIntersection(a, b);
                if (northIntersection != null) return (localTileCoordinate + Vector2Int.up, northIntersection.Value);
                
                // If intersection is not at the North edge, then it must be at the East edge
                var eastIntersection = GetTileEdge(localTileCoordinate, TileEdgeSide.East).GetIntersection(a, b);
                return (localTileCoordinate + Vector2Int.right, eastIntersection!.Value);
            }

            if (direction < 180f)
            {
                // The line must either exit the North edge or West face
                // Check north first
                var northIntersection = GetTileEdge(localTileCoordinate, TileEdgeSide.North).GetIntersection(a, b);
                if (northIntersection != null) return (localTileCoordinate + Vector2Int.up, northIntersection.Value);
                
                // If intersection is not at the North edge, then it must be at the West edge
                var westIntersection = GetTileEdge(localTileCoordinate, TileEdgeSide.West).GetIntersection(a, b);
                return (localTileCoordinate + Vector2Int.left, westIntersection!.Value);          
            }

            if (direction < 270f)
            {
                // The line must either exit the West edge or South edge
                // Check West first
                var westIntersection = GetTileEdge(localTileCoordinate, TileEdgeSide.West).GetIntersection(a, b);
                if (westIntersection != null) return (localTileCoordinate + Vector2Int.left, westIntersection.Value);
                
                // If intersection is not at the West edge, then it must be at the South edge
                var southIntersection = GetTileEdge(localTileCoordinate, TileEdgeSide.South).GetIntersection(a, b);
                return (localTileCoordinate + Vector2Int.down, southIntersection!.Value);
            }

            else
            {
                // The line must either exit the South edge or East edge
                // Check South first
                var southIntersection = GetTileEdge(localTileCoordinate, TileEdgeSide.South).GetIntersection(a, b);
                if (southIntersection != null) return (localTileCoordinate + Vector2Int.down, southIntersection.Value);
                
                // If intersection is not at the South edge, then it must be at the East edge
                var eastIntersection = GetTileEdge(localTileCoordinate, TileEdgeSide.East).GetIntersection(a, b);
                return (localTileCoordinate + Vector2Int.right, eastIntersection!.Value);
            }
                
        }

        private AxialLine2D GetTileEdge(Vector2Int tileCoordinate, TileEdgeSide side)
        {
            return _tileEdges[tileCoordinate.x, tileCoordinate.y, (int) side];
        }

        public int TriangleCount()
        {
            return WidthInTiles * HeightInTiles * 8;
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
                   && localCoordinates.y >= 0.0f && localCoordinates.y < HeightInTiles; 
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