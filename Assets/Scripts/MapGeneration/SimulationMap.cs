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
        
        // The tiles of the map (each tile containing 8 triangle cells) 
        private readonly SimulationMapTile<TCell>[,] _tiles;

        // Used for calculating ray traces on the map. Lazily initialized.
        private RayTracingTriangle[] _traceableTriangles;
        
        public SimulationMap(Functional.Factory<TCell> cellFactory, int widthInTiles, int heightInTiles, float scale, Vector2 scaledOffset)
        {
            this.ScaledOffset = scaledOffset;
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
        }
        
        // Private constructor for a pre-specified set of tiles. This is used in the FMap function
        private SimulationMap(SimulationMapTile<TCell>[,] tiles, float scale, Vector2 scaledOffset)
        {
            this.ScaledOffset = scaledOffset;
            _tiles = tiles;
            this.Scale = scale;
            WidthInTiles = tiles.GetLength(0);
            HeightInTiles = tiles.GetLength(1);
        }


        // Casts a trace starting at given point, moving in the given direction. The given function will be called on
        // each cell that is encountered. If the function returns true the trace will continue to the next cell,
        // if it returns false the trace will terminate. The trace automatically terminates when it exits map bounds.
        public void Raytrace(Vector2 startingPoint, float angleDegrees, float distance, Func<TCell, bool> shouldContinueFromCell)
        {
            if (_traceableTriangles == null) GenerateTraceableTriangles();
            int nextTriangleIndex = GetTriangleIndex(startingPoint);
            
            // Convert given angle and starting point to a linear equation: ax + b
            var a = Mathf.Tan(Mathf.PI / 180 * angleDegrees);
            var b = startingPoint.y - a * startingPoint.x;

            var currentTriangle = _traceableTriangles[nextTriangleIndex];
            var enteringEdge = currentTriangle.FindInitialEnteringEdge(angleDegrees, a, b);

            while(true)
            {
                // Invoke the given function on the cell, and only continue if it returns true
                if (!shouldContinueFromCell(currentTriangle.Cell))
                    break;

                // Perform the ray tracing step for the current triangle
                (enteringEdge, _, nextTriangleIndex) = currentTriangle.RayTrace(enteringEdge, a, b);
                
                // Break if the next triangle is outside the map bounds
                if (nextTriangleIndex < 0 || nextTriangleIndex >= _traceableTriangles.Length)
                    break; 

                currentTriangle = _traceableTriangles[nextTriangleIndex];

                // All vertices of the triangle must be within range for the triangle to be considered visible
                bool withinRange = Geometry.DistanceBetween(startingPoint, currentTriangle._lines[0].Start) <= distance;
                withinRange &= Geometry.DistanceBetween(startingPoint, currentTriangle._lines[1].Start) <= distance;
                withinRange &= Geometry.DistanceBetween(startingPoint, currentTriangle._lines[2].Start) <= distance;
                if (!withinRange)
                    break;
            }
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
        private int GetTriangleIndex(Vector2 coordinate)
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
                                            + " is not within map bounds: {" + WidthInTiles + ", " + HeightInTiles + "}");
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
            {
                for (int y = 0; y < HeightInTiles; y++)
                {
                    mappedTiles[x, y] = _tiles[x, y].FMap(mapper);
                }
            }
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

        
        // ---------------------- Custom "Raytracing" -------------------------------

        // Initializes 
        private void GenerateTraceableTriangles()
        {
            var totalTriangles = WidthInTiles * HeightInTiles * 8;
            var trianglesPerRow = WidthInTiles * 8;
            var vertexDistance = Scale / 2.0f;
            
            _traceableTriangles = new RayTracingTriangle[totalTriangles];
            for (int x = 0; x < WidthInTiles; x++)
            {
                for (int y = 0; y < HeightInTiles; y++)
                {
                    int index = x * 8 + y * trianglesPerRow;
                    AddTraceableTriangles(new Vector2(x, y) * Scale + ScaledOffset, vertexDistance, index, trianglesPerRow);
                }
            }

            foreach (var (index, cell) in this)
            {
                _traceableTriangles[index].Cell = cell;
            }
        }

        // Sorry about this function...
        // Adds 8 'Traceable triangles' for a given tile. These triangles contain information that allows effective 
        // raytracing. Each triangle contains a list of their neighbours indexes. Additionally it contains information
        // about the 3 lines that make up the triangle (An inclined line, a horizontal line and a vertical line
        // (in that order)
        private void AddTraceableTriangles(Vector2 bottomLeft, float vertexDistance, int index, int trianglesPerRow)
        {
            var x = bottomLeft.x;
            var y = bottomLeft.y;
            // Triangle 0
            var neighbours = new int[3];
            neighbours[Inclined] = index + 1;
            neighbours[Horizontal] = index - trianglesPerRow + 4;
            neighbours[Vertical] = index - 5;
            _traceableTriangles[index] = new RayTracingTriangle(
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
            _traceableTriangles[index] = new RayTracingTriangle(
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
            _traceableTriangles[index] = new RayTracingTriangle(
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
            _traceableTriangles[index] = new RayTracingTriangle(
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
            _traceableTriangles[index] = new RayTracingTriangle(
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
            _traceableTriangles[index] = new RayTracingTriangle(
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
            _traceableTriangles[index] = new RayTracingTriangle(
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
            _traceableTriangles[index] = new RayTracingTriangle(
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
            public Line2D[] _lines;
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

            // When starting a ray trace, it must be determined which of the 3 edges are to be considered to the
            // initial "entering" edge
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