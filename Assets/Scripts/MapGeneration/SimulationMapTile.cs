using System;
using System.Collections.Generic;
using System.Linq;
using Dora.Utilities;
using UnityEngine;

namespace Dora.MapGeneration
{
    public class SimulationMapTile<TCell>
        {
            // A tile is a rectangle consisting of 8 triangle shaped cells.
            // The triangles are arranged in 4 different orientations
            // The triangles are indexed and arranged as shown in this very pretty illustration:
            ///  |4/5|6\7|
            ///  |0\1|2/3|
            private List<TCell> _triangleCells = new List<TCell>();
            
            public SimulationMapTile(Functional.Factory<TCell> cellFactory)
            {
                for (int i = 0; i < 8; i++)
                {
                    _triangleCells.Add(cellFactory());
                    
                }
            }

            public void SetCellValue(int index, TCell newCellValue)
            {
                this._triangleCells[index] = newCellValue;
            }

            private SimulationMapTile(List<TCell> cells)
            {
                this._triangleCells = cells;
            }
            
            
            public SimulationMapTile<TNewCell> FMap<TNewCell>(Func<TCell,TNewCell> mapper)
            {
                List<TNewCell> mappedCells = new List<TNewCell>();
                foreach (var cell in _triangleCells)
                {
                    mappedCells.Add(mapper(cell));
                }
                return new SimulationMapTile<TNewCell>(mappedCells);
            }
            
            public void ForEachCell(Action<TCell> cellAction)
            {
                foreach (var cell in _triangleCells)
                {
                    cellAction(cell);
                }
            }

            public TCell GetTriangleCellByCoordinateDecimals(float xDecimals, float yDecimals)
            {
                return _triangleCells[CoordinateDecimalsToTriangleIndex(xDecimals, yDecimals)];
            }

            public void SetTriangleCellByCoordinateDecimals(float xDecimals, float yDecimals, TCell newCell)
            {
                _triangleCells[CoordinateDecimalsToTriangleIndex(xDecimals, yDecimals)] = newCell;
            }
            
            
            private int CoordinateDecimalsToTriangleIndex(float xDecimal, float yDecimal)
            {
                if (xDecimal < 0.0f || xDecimal > 1.0f || yDecimal < 0.0f || yDecimal > 1.0f)
                    throw new ArgumentException("Coordinate decimals must be between 0.0 and 1.0. " +
                                                "Coordinates were: (" + xDecimal + ", " + yDecimal + " )");

                var index = 0;

                if (yDecimal < 0.5)
                {
                    // Bottom left quadrant
                    if (xDecimal + yDecimal < 0.5f) return 0;
                    if (xDecimal + yDecimal >= 0.5f && xDecimal < 0.5f) return 1;
                    // Bottom right quadrant
                    if (xDecimal - yDecimal < 0.5f) return 2;
                    return 3;
                }
                else
                {
                    // Top left quadrant
                    if (yDecimal - xDecimal > 0.5f && xDecimal < 0.5f) return 4;
                    if (yDecimal - xDecimal <= 0.5f && xDecimal < 0.5f) return 5;
                    // Top right quadrant
                    if (xDecimal + yDecimal < 1.5f) return 6;
                    return 7;
                }
            }

            public List<TCell> GetTriangles()
            {
                return _triangleCells;
            }
            
            
            private enum TriangleEdge
            {
                Inclined=0, Horizontal=1, Vertical=2
            }

            private static TriangleEdge[] _triangleEdges = new[]
                {TriangleEdge.Inclined, TriangleEdge.Horizontal, TriangleEdge.Vertical};

            private class RayTracingTriangle
            {
                private Line2D[] _lines;
                private int[] _neighbourIndex;

                public RayTracingTriangle(Line2D[] lines, int[] neighbourIndex)
                {
                    _lines = lines;
                    _neighbourIndex = neighbourIndex;
                }

                // Returns the side at which the trace exited the triangle, the exit intersection point
                // and the index of the triangle that the trace enters next
                // Takes the edge that this tile was entered from, and the linear equation ax+b for the trace 
                public (TriangleEdge, Vector2, int) RayTrace(TriangleEdge enteringEdge, float a, float b)
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
            }
            
        }
}