using System.Collections.Generic;
using Dora.MapGeneration;
using UnityEngine;

namespace Dora.Statistics
{
    public class ExplorationVisualizer: MonoBehaviour
    {

        public MeshRenderer meshRenderer;
        public MeshFilter meshFilter;
        private Mesh mesh;

        private readonly Color _solidColor = new Color(0.0f, 0.0f, 0.0f);
        private readonly Color _exploredColor = new Color(0.2f, 0.8f, 0.5f);
        private readonly Color _unexploredColor = new Color(0.5f, 0.5f, 0.5f);
        
        
        private int _widthInTiles, _heightInTiles;
        private int _widthInVertices, _heightInVertices;
        private float _scale;
        private Vector3 _offset;

        private List<Vector3> _vertices = new List<Vector3>();
        private List<int> _triangles = new List<int>();
        private Color[] _colors;

        private const int ResolutionMultiplier = 2;
        private SimulationMap<ExplorationCell> _map;

        public void SetMap(SimulationMap<ExplorationCell> newMap, float scale, Vector3 offset)
        {
            _map = newMap;
            _widthInTiles = _map.Width;
            _heightInTiles = _map.Height;
            _scale = scale;
            _offset = offset;
            _widthInVertices = _widthInTiles * ResolutionMultiplier + 1;
            _heightInVertices = _heightInTiles * ResolutionMultiplier + 1;
            
            GenerateVertices();
            GenerateTriangles();
            _colors = new Color[_map.TriangleCount()];
            UpdateColors(_map);
            
            mesh = new Mesh();
            mesh.vertices = _vertices.ToArray();
            mesh.triangles = _triangles.ToArray();
            mesh.RecalculateNormals();
            //mesh.colors = _colors;

            meshFilter.mesh = mesh;
        }

      
        
        private void UpdateColors(SimulationMap<ExplorationCell> newMap)
        {
            
            foreach (var (index, explorationCell) in newMap)
            {
                _colors[index] = explorationCell.isExplorable ? _unexploredColor : _solidColor;
            }
        }

        private void GenerateVertices()
        {
            _vertices.Clear();
            float coordinateMultiplier = _scale / ResolutionMultiplier;
            for (int y = 0; y < _heightInVertices; y++)
            {
                for (int x = 0; x < _widthInVertices; x++)
                {
                    _vertices.Add(new Vector3(x * coordinateMultiplier, y * coordinateMultiplier, 0f));
                }                
            }
        }

        private void GenerateTriangles()
        {
            _triangles.Clear();
            for (int y = 0; y < _heightInTiles; y++)
            {
                for (int x = 0; x < _widthInTiles; x++)
                {
                    // The index of the first vertex in the bottom row of the tile
                    var row0Index = y * ResolutionMultiplier * _widthInVertices + x * ResolutionMultiplier;
                    // Index of first vertex in second row of the tile
                    var row1Index = row0Index + _widthInVertices;
                    // Index of first vertex in third row of the tile
                    var row2Index = row1Index + _widthInVertices;
                    
                    // Add the 8 triangles of this tile to the triangle list
                    AddTriangles(row0Index, row1Index, row2Index);
                }                
            }
        }

        // Adds the 8 triangles of a tile starting at the given offsets
        // The triangles are indexed and arranged as shown in this very pretty illustration:
        // |4/5|6\7|
        // |0\1|2/3|
        private void AddTriangles(int row0, int row1, int row2)
        {
            // Triangle 0
            _triangles.Add(row0);
            _triangles.Add(row1);
            _triangles.Add(row0 + 1);
            
            // Triangle 1
            _triangles.Add(row1);
            _triangles.Add(row1 + 1);
            _triangles.Add(row0 + 1);
            
            // Triangle 2
            _triangles.Add(row1 + 1);
            _triangles.Add(row1 + 2);
            _triangles.Add(row0 + 1);

            // Triangle 3
            _triangles.Add(row0 + 1);
            _triangles.Add(row1 + 2);
            _triangles.Add(row0 + 2);
            
            // Triangle 4
            _triangles.Add(row1);
            _triangles.Add(row2);
            _triangles.Add(row2 + 1);
            
            // Triangle 5
            _triangles.Add(row1);
            _triangles.Add(row2 + 1);
            _triangles.Add(row1 + 1);
            
            // Triangle 6
            _triangles.Add(row1 + 1);
            _triangles.Add(row2 + 1);
            _triangles.Add(row1 + 2);
            
            // Triangle 7
            _triangles.Add(row2 + 1);
            _triangles.Add(row2 + 2);
            _triangles.Add(row1 + 2);
        }
    }
}