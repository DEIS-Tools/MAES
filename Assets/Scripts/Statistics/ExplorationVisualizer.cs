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
        private Vector3 _scaledOffset;

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
            _scaledOffset = offset;
            _widthInVertices = _widthInTiles * ResolutionMultiplier + 1;
            _heightInVertices = _heightInTiles * ResolutionMultiplier + 1;
            
            //GenerateVertices();
            GenerateTriangleVertices();
            GenerateTriangles();
            _colors = new Color[_vertices.Count];
            UpdateColors(_map);
            
            mesh = new Mesh();
            mesh.vertices = _vertices.ToArray();
            mesh.triangles = _triangles.ToArray();
            mesh.RecalculateNormals();
            
            mesh.colors = _colors;

            meshFilter.mesh = mesh;
        }

        private void GenerateTriangles()
        {
            // The vertices are already arranged in the correct order (ie. triangle 0 has vertices indexed 0, 1, 2)
            _triangles = new List<int>();
            for (int i = 0; i < _vertices.Count; i++)
                _triangles.Add(i);
        }


        // Colors each triangle depending on its current state
        private void UpdateColors(SimulationMap<ExplorationCell> newMap)
        {
            foreach (var (index, explorationCell) in newMap)
            {
                var vertexIndex = index * 3;
                var color = explorationCell.isExplorable ? _unexploredColor : _solidColor;
                _colors[index] = color;
                _colors[index + 1] = color;
                _colors[index + 2] = color;
            }
        }

        private void GenerateTriangleVertices()
        {
            _vertices.Clear();
            var vertexDistance = _scale / ResolutionMultiplier;
            for (int y = 0; y < _heightInTiles; y++)
            {
                var translatedY = y * _scale;
                for (int x = 0; x < _widthInTiles; x++)
                {
                    var translatedX = x * _scale;
                    AddTileTriangleVertices(translatedX, translatedY, vertexDistance);
                }
            }
        }

        // Adds all of the vertices needed for a tile of 8 triangles
        // The triangles are indexed and arranged as shown in this very pretty illustration:
        // |4/5|6\7|
        // |0\1|2/3|
        private void AddTileTriangleVertices(float x, float y, float vertexDistance)
        {
            // Triangle 0
            _vertices.Add(new Vector3(x, y, 0f));
            _vertices.Add(new Vector3(x, y + vertexDistance, 0f));
            _vertices.Add(new Vector3(x + vertexDistance, y, 0f));
            
            // Triangle 1
            _vertices.Add(new Vector3(x, y + vertexDistance));
            _vertices.Add(new Vector3(x + vertexDistance, y + vertexDistance));
            _vertices.Add(new Vector3(x + vertexDistance, y));
            
            // Triangle 2
            _vertices.Add(new Vector3(x + vertexDistance, y + vertexDistance));
            _vertices.Add(new Vector3(x + 2 * vertexDistance, y +  vertexDistance));
            _vertices.Add(new Vector3(x + vertexDistance, y));
            
            // Triangle 3
            _vertices.Add(new Vector3(x + vertexDistance, y));
            _vertices.Add(new Vector3(x + 2 * vertexDistance, y + vertexDistance));
            _vertices.Add(new Vector3(x + 2 * vertexDistance, y));
            
            // Triangle 4
            _vertices.Add(new Vector3(x, y + 2 * vertexDistance));
            _vertices.Add(new Vector3(x + vertexDistance, y + 2 * vertexDistance));
            _vertices.Add(new Vector3(x, y + vertexDistance));
            
            // Triangle 5
            _vertices.Add(new Vector3(x, y + vertexDistance));
            _vertices.Add(new Vector3(x + vertexDistance, y + 2 * vertexDistance));
            _vertices.Add(new Vector3(x + vertexDistance, y + vertexDistance));
            
            // Triangle 6
            _vertices.Add(new Vector3(x + vertexDistance, y + 2 *vertexDistance));
            _vertices.Add(new Vector3(x + 2 * vertexDistance, y + vertexDistance));
            _vertices.Add(new Vector3(x + vertexDistance, y + vertexDistance));
            
            // Triangle 7
            _vertices.Add(new Vector3(x + vertexDistance, y + 2 * vertexDistance));
            _vertices.Add(new Vector3(x + 2 * vertexDistance, y + 2 * vertexDistance));
            _vertices.Add(new Vector3(x + 2 * vertexDistance, y + vertexDistance));
        }
    }
}