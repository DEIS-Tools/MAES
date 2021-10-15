using System.Collections.Generic;
using UnityEngine;

namespace Dora.Statistics
{
    public class ExplorationVisualizer: MonoBehaviour
    {

        public MeshRenderer meshRenderer;
        public MeshFilter meshFilter;
        private Mesh mesh;
        
        
        private int _widthInTiles, _heightInTiles;
        private int _widthInVertices, _heightInVertices;
        private float _scale;
        private Vector3 _offset;

        private List<Vector3> _vertices = new List<Vector3>();
        private List<int> _triangles = new List<int>();

        private const int ResolutionMultiplier = 2;

        public void SetMap(int widthInTiles, int heightInTiles, float scale, Vector3 offset)
        {
            _widthInTiles = widthInTiles;
            _heightInTiles = heightInTiles;
            _scale = scale;
            _offset = offset;
            _widthInVertices = widthInTiles * ResolutionMultiplier + 1;
            _heightInVertices = heightInTiles * ResolutionMultiplier + 1;
            GenerateVertices();
            GenerateTriangles();
            
            mesh = new Mesh();
            mesh.vertices = _vertices.ToArray();
            mesh.triangles = _triangles.ToArray();
            mesh.RecalculateNormals();

            meshFilter.mesh = mesh;
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
            for (int y = 0; y < _heightInVertices; y++)
            {
                for (int x = 0; x < _widthInVertices; x++)
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
            _triangles.Add(row0 + 1);
            _triangles.Add(row1 + 2);
            
            // Triangle 3
            _triangles.Add(row0 + 1);
            _triangles.Add(row1 + 2);
            _triangles.Add(row0 + 2);
            
            // Triangle 4
            _triangles.Add(row1);
            _triangles.Add(row2);
            _triangles.Add(row2 + 1);
            
            // Triangle 5
            _triangles.Add(row2 + 1);
            _triangles.Add(row1);
            _triangles.Add(row1 + 1);
            
            // Triangle 6
            _triangles.Add(row2 + 1);
            _triangles.Add(row1 + 1);
            _triangles.Add(row1 + 2);
            
            // Triangle 7
            _triangles.Add(row2 + 1);
            _triangles.Add(row2 + 2);
            _triangles.Add(row1 + 2);
        }
    }
}