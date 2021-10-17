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
        private Line2D[,,] _tileEdges;

        private RayTracingTriangle[] _tracableTriangles;
        
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
            _tileEdges = new Line2D[widthInTiles, heightInTiles, 4];
            for (int x = 0; x < widthInTiles; x++)
            {
                for (int y = 0; y < heightInTiles; y++)
                {
                    var v1 = new Vector2(x, y) * Scale + Offset;
                    var v2 = new Vector2(x + 1, y)  * Scale + Offset;
                    var v3 = new Vector2(x + 1, y + 1)  * Scale + Offset;
                    var v4 = new Vector2(x, y + 1)  * Scale + Offset;
                    _tileEdges[x, y, (int) TileEdgeSide.South] = new Line2D(v1, v2);
                    _tileEdges[x, y, (int) TileEdgeSide.East] = new Line2D(v2, v3);
                    _tileEdges[x, y, (int) TileEdgeSide.North] = new Line2D(v4, v3);
                    _tileEdges[x, y, (int) TileEdgeSide.West] = new Line2D(v1, v4);
                }
            }
        }

        // Casts a trace starting at given point, moving in the given direction and terminating when encountering a
        // collision or after exceeding the given direction.
        // Returns the list of intersected cells in the order that they were encountered  
        public List<TCell> Raytrace(Vector2 startingPoint, float angleDegrees, float distance, Func<TCell, bool> shouldContinueFromCell)
        {
            if (_tracableTriangles == null) GenerateTraceableTriangles();
            int nextTriangleIndex = GetTriangleIndex(startingPoint);
            
            // Convert angle to a
            // solve for b
            var a = Mathf.Tan(Mathf.PI / 180 * angleDegrees);
            var b = startingPoint.y - a * startingPoint.x;

            var currentTriangle = _tracableTriangles[nextTriangleIndex];
            var enteringEdge = currentTriangle.FindInitialEnteringEdge(angleDegrees, a, b);
            Vector2 intersectionPoint;
            
            do
            {
                currentTriangle = _tracableTriangles[nextTriangleIndex];
                if (!shouldContinueFromCell(currentTriangle.Cell))
                    break;
                (enteringEdge, intersectionPoint, nextTriangleIndex) = currentTriangle.RayTrace(enteringEdge, a, b);
            } while (Geometry.DistanceBetween(startingPoint, intersectionPoint) <= distance 
                     && nextTriangleIndex > 0 && nextTriangleIndex < _tracableTriangles.Length);
            
            /*_tracableTriangles[triangleIndex].RayTrace(enteringEdge, a, b);

            // First find the Tiles that intersect with the trace
            var tracedTiles = TilesTrace(startingPoint, angleDegrees, distance);
            
            var tracedCells = new List<TCell>();
            foreach (var tile in tracedTiles)
            {
                tile.ForEachCell(cellAction);
            }*/
            var tracedCells = new List<TCell>();
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

        private Line2D GetTileEdge(Vector2Int tileCoordinate, TileEdgeSide side)
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
        
        // Returns the index of triangle cell at the given world position
        private int GetTriangleIndex(Vector2 coordinate)
        {
            var localCoordinate = ToLocalMapCoordinate(coordinate);
            var tile = _tiles[(int) localCoordinate.x, (int) localCoordinate.y];
            var triangleIndexOffset = ((int) localCoordinate.x) * 8 + ((int) localCoordinate.y) * WidthInTiles * 8;
            return triangleIndexOffset + 
                   tile.CoordinateDecimalsToTriangleIndex(localCoordinate.x % 1.0f, localCoordinate.y % 1.0f);
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
        
        // ---------------------- Custom "Raytracing" -------------------------------

        // Initializes 
        private void GenerateTraceableTriangles()
        {
            var totalTriangles = WidthInTiles * HeightInTiles * 8;
            var trianglesPerRow = WidthInTiles * 8;
            var vertexDistance = Scale / 2.0f;
            
            _tracableTriangles = new RayTracingTriangle[totalTriangles];
            for (int x = 0; x < WidthInTiles; x++)
            {
                for (int y = 0; y < HeightInTiles; y++)
                {
                    int index = x * 8 + y * trianglesPerRow;
                    AddTraceableTriangles(new Vector2(x, y) * Scale + Offset, vertexDistance, index, trianglesPerRow);
                }
            }

            foreach (var (index, cell) in this)
            {
                _tracableTriangles[index].Cell = cell;
            }
        }

        private void AddTraceableTriangles(Vector2 bottomLeft, float vertexDistance, int index, int trianglesPerRow)
        {
            var x = bottomLeft.x;
            var y = bottomLeft.y;
            // Triangle 0
            var neighbours = new int[3];
            neighbours[Inclined] = index + 1;
            neighbours[Horizontal] = index - trianglesPerRow + 4;
            neighbours[Vertical] = index - 5;
            _tracableTriangles[index] = new RayTracingTriangle(
                new Vector2(x, y + vertexDistance), 
                new Vector2(x + vertexDistance, y),
                new Vector2(x, y),
                neighbours
            );

            // Triangle 1
            index++;
            var neighbours1 = new int[3];
            neighbours1[Inclined] = index - 1;
            neighbours1[Horizontal] = index + 4;
            neighbours1[Vertical] = index + 1;
            _tracableTriangles[index] = new RayTracingTriangle(
                new Vector2(x + vertexDistance, y), 
                new Vector2(x, y + vertexDistance),
                new Vector2(x + vertexDistance, y + vertexDistance),
                neighbours1
            );
            
            // Triangle 2
            index++;
            var neighbours2 = new int[3];
            neighbours2[Inclined] = index + 1;
            neighbours2[Horizontal] = index + 4;
            neighbours2[Vertical] = index - 1;
            _tracableTriangles[index] = new RayTracingTriangle(
                new Vector2(x + vertexDistance, y),
                new Vector2(x + 2 * vertexDistance, y + vertexDistance),
                new Vector2(x + vertexDistance, y + vertexDistance),
                neighbours2
            );
            
            // Triangle 3
            index++;
            var neighbours3 = new int[3];
            neighbours3[Inclined] = index - 1;
            neighbours3[Horizontal] = index - trianglesPerRow + 4;
            neighbours3[Vertical] = index + 5;
            _tracableTriangles[index] = new RayTracingTriangle(
                new Vector2(x + 2 * vertexDistance, y + vertexDistance),
                new Vector2(x + vertexDistance, y),
                new Vector2(x + 2 * vertexDistance, y),
                neighbours3
            );
            
            // Triangle 4
            index++;
            var neighbours4 = new int[3];
            neighbours4[Inclined] = index + 1;
            neighbours4[Horizontal] = index + trianglesPerRow - 4;
            neighbours4[Vertical] = index - 5;
            _tracableTriangles[index] = new RayTracingTriangle(
                new Vector2(x, y + vertexDistance),
                new Vector2(x + vertexDistance, y + 2 * vertexDistance),
                new Vector2(x, y + 2 * vertexDistance),
                neighbours4
            );
            
            // Triangle 5
            index++;
            var neighbours5 = new int[3];
            neighbours5[Inclined] = index - 1;
            neighbours5[Horizontal] = index - 4;
            neighbours5[Vertical] = index + 1;
            _tracableTriangles[index] = new RayTracingTriangle(
                new Vector2(x + vertexDistance, y + 2 * vertexDistance),
                new Vector2(x, y + vertexDistance),
                new Vector2(x + vertexDistance, y + vertexDistance),
                neighbours5
            );
            
            // Triangle 6
            index++;
            var neighbours6 = new int[3];
            neighbours6[Inclined] = index + 1;
            neighbours6[Horizontal] = index - 4;
            neighbours6[Vertical] = index - 1;
            _tracableTriangles[index] = new RayTracingTriangle(
                new Vector2(x + vertexDistance, y + 2 * vertexDistance),
                new Vector2(x + 2 * vertexDistance, y + vertexDistance),
                new Vector2(x + vertexDistance, y + vertexDistance),
                neighbours6
            );
            
            // Triangle 7
            index++;
            var neighbours7 = new int[3];
            neighbours7[Inclined] = index - 1;
            neighbours7[Horizontal] = index + trianglesPerRow - 4;
            neighbours7[Vertical] = index + 5;
            _tracableTriangles[index] = new RayTracingTriangle(
                new Vector2(x + 2 * vertexDistance, y + vertexDistance),
                new Vector2(x + vertexDistance, y + 2 * vertexDistance),
                new Vector2(x + 2 * vertexDistance, y + 2 * vertexDistance),
                neighbours7
            );
        }
        
        private const int Inclined = 0, Horizontal = 1, Vertical = 2;
        private static readonly int[] _triangleEdges = new[]
            {Inclined, Horizontal, Vertical};

        private class RayTracingTriangle
        {
            private Line2D[] _lines;
            private int[] _neighbourIndex;
            public TCell Cell;

            public RayTracingTriangle(Vector2 p1, Vector2 p2, Vector2 p3, int[] neighbourIndex)
            {
                _lines = new Line2D[3]{new Line2D(p1, p2), new Line2D(p2, p3), new Line2D(p3, p1)};
                _neighbourIndex = neighbourIndex;
            }

            // Returns the side at which the trace exited the triangle, the exit intersection point
            // and the index of the triangle that the trace enters next
            // Takes the edge that this tile was entered from, and the linear equation ax+b for the trace 
            public (int, Vector2, int) RayTrace(int enteringEdge, float a, float b)
            {
                foreach (var edge in _triangleEdges)
                {
                    // The line must exit the triangle in one of the two edges that the line did not enter through
                    // Therefore only check intersection for these two lines
                    if (edge == enteringEdge) continue;
                        
                    var intersection = _lines[(int) edge].GetIntersection(a, b);
                    if (intersection != null) return (edge, intersection!.Value, _neighbourIndex[(int) edge]);
                }

                throw new Exception("Triangle does not have any intersections with the given line");
            }

            // When starting a ray trace, it must be determined which of the 3 edges are to be considered to the intial
            // entering edge
            public int FindInitialEnteringEdge(float direction, float a, float b)
            {
                List<(Vector2, int)> intersectionsAndEdge = new List<(Vector2, int)>();
                foreach (var edge in _triangleEdges)
                {
                    var intersection = _lines[edge].GetIntersection(a, b);
                    if (intersection != null) intersectionsAndEdge.Add((intersection!.Value, edge));
                }

                var intersectionOneX = intersectionsAndEdge[0].Item1.x;
                var intersectionTwoX = intersectionsAndEdge[1].Item1.x;
                if (direction <= 90 || direction >= 270)
                {
                    // Entering point must be the left most intersection
                    return intersectionOneX < intersectionTwoX
                        ? intersectionsAndEdge[0].Item2
                        : intersectionsAndEdge[1].Item2;
                }
                else
                {
                    // Entering point must be the right most intersection
                    return intersectionOneX > intersectionTwoX
                        ? intersectionsAndEdge[0].Item2
                        : intersectionsAndEdge[1].Item2;
                }
            }
        }
        
    }
    
    
    
    
}