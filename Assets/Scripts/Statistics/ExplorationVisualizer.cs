using System.Collections.Generic;
using Dora.MapGeneration;
using UnityEngine;
using UnityEngine.Rendering;

namespace Dora.Statistics
{
    public class ExplorationVisualizer: MonoBehaviour
    {

        public MeshRenderer meshRenderer;
        public MeshFilter meshFilter;
        private Mesh mesh;

        private readonly Color32 _solidColor = new Color32(0, 0, 0, 255);
        private readonly Color32 _exploredColor = new Color32(50, 220, 126, 255);
        private readonly Color32 _unexploredColor = new Color32(125, 125, 125, 255);

        private int _widthInTiles, _heightInTiles;
        private int _widthInVertices, _heightInVertices;
        private float _scale;
        private Vector3 _scaledOffset;

        private List<Vector3> _vertices = new List<Vector3>();
        private List<int> _triangles = new List<int>();
        private Color32[] _colors;

        private const int ResolutionMultiplier = 2;
        private SimulationMap<ExplorationCell> _map;

        public void SetMap(SimulationMap<ExplorationCell> newMap, float scale, Vector3 offset)
        {
            _map = newMap;
            _widthInTiles = _map.WidthInTiles;
            _heightInTiles = _map.HeightInTiles;
            _scale = scale;
            _scaledOffset = offset;
            _widthInVertices = _widthInTiles * ResolutionMultiplier + 1;
            _heightInVertices = _heightInTiles * ResolutionMultiplier + 1;
            
            //GenerateVertices();
            GenerateTriangleVertices();
            GenerateTriangles();
            _colors = new Color32[_vertices.Count];
            InitializeColors(_map);
            
            mesh = new Mesh();
            mesh.indexFormat = IndexFormat.UInt32;
            mesh.vertices = _vertices.ToArray();
            mesh.triangles = _triangles.ToArray();
            mesh.RecalculateNormals();
            
            mesh.colors32 = _colors;
            meshFilter.mesh = mesh;
        }

        private void GenerateTriangleVertices()
        {
            _vertices.Clear();
            var vertexDistance = _scale / ResolutionMultiplier;
            for (int y = 0; y < _heightInTiles; y++)
            {
                var translatedY = y * _scale + _scaledOffset.y;
                for (int x = 0; x < _widthInTiles; x++)
                {
                    var translatedX = x * _scale + _scaledOffset.x;
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
        
        
        private void GenerateTriangles()
        {
            _triangles.Clear();
            // The vertices are already arranged in the correct order (ie. triangle 0 has vertices indexed 0, 1, 2)
            _triangles = new List<int>();
            for (int i = 0; i < _vertices.Count; i++)
                _triangles.Add(i);
        }
        
        // Colors each triangle depending on its current state
        public void InitializeColors(SimulationMap<ExplorationCell> newMap)
        {
            int count = 0;
            foreach (var (index, explorationCell) in newMap)
            {
                var vertexIndex = index * 3;
                var color = _solidColor;
                if (explorationCell.isExplorable)
                    color = explorationCell.IsExplored ? _exploredColor : _unexploredColor;
                _colors[vertexIndex] = color;
                _colors[vertexIndex + 1] = color;
                _colors[vertexIndex + 2] = color;
                if (!explorationCell.isExplorable) count++;
            }
        }
        
        // Colors each triangle depending on its current state
        public void SetExplored(List<int> triangles)
        {
            foreach (var index in triangles)
            {
                var vertexIndex = index * 3;
                _colors[vertexIndex] = _exploredColor;
                _colors[vertexIndex + 1] = _exploredColor;
                _colors[vertexIndex + 2] = _exploredColor;
            }
            
            mesh.colors32 = _colors;
        }
    }
}