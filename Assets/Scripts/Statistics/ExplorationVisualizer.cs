using System.Collections.Generic;
using Maes.Map;
using Maes.Robot;
using UnityEngine;
using UnityEngine.Rendering;

namespace Maes.Statistics {
    public class ExplorationVisualizer : MonoBehaviour {
        public MeshRenderer meshRenderer;
        public MeshFilter meshFilter;
        private Mesh mesh;

        public static readonly Color32 SolidColor = new Color32(0, 0, 0, 255);
        public static readonly Color32 ExploredColor = new Color32(32, 130, 57, 255);
        public static readonly Color32 StandardCellColor = new Color32(170, 170, 170, 255);
        public static readonly Color32 CoveredColor = new Color32(32, 80, 240, 255);
        public static readonly Color32 SlamSeenColor = new Color32(50, 120, 180, 255);
        public static readonly Color32 WarmColor = new Color32(200, 60, 60, 255);
        public static readonly Color32 ColdColor = new Color32(50, 120, 180, 255);

        private int _widthInTiles, _heightInTiles;
        private int _widthInVertices, _heightInVertices;
        private Vector3 _offset;

        private List<Vector3> _vertices = new List<Vector3>();
        private List<int> _triangles = new List<int>();
        private Color32[] _colors;

        private const int ResolutionMultiplier = 2;
        private SimulationMap<ExplorationCell> _map;
        
        public delegate Color32 CellToColor(ExplorationCell cell);
        public delegate Color32 CellIndexToColor(int cellIndex);
        

        public void SetMap(SimulationMap<ExplorationCell> newMap, Vector3 offset) {
            _map = newMap;
            _widthInTiles = _map.WidthInTiles;
            _heightInTiles = _map.HeightInTiles;
            _offset = offset;
            _widthInVertices = _widthInTiles * ResolutionMultiplier + 1;
            _heightInVertices = _heightInTiles * ResolutionMultiplier + 1;

            // GenerateVertices();
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

        private void GenerateTriangleVertices() {
            _vertices.Clear();
            var vertexDistance = 1f / ResolutionMultiplier;
            for (int y = 0; y < _heightInTiles; y++) {
                var translatedY = y + _offset.y;
                for (int x = 0; x < _widthInTiles; x++) {
                    var translatedX = x + _offset.x;
                    AddTileTriangleVertices(translatedX, translatedY, vertexDistance);
                }
            }
        }

        // Adds all of the vertices needed for a tile of 8 triangles
        // The triangles are indexed and arranged as shown in this very pretty illustration:
        // |4/5|6\7|
        // |0\1|2/3|
        private void AddTileTriangleVertices(float x, float y, float vertexDistance) {
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
            _vertices.Add(new Vector3(x + 2 * vertexDistance, y + vertexDistance));
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
            _vertices.Add(new Vector3(x + vertexDistance, y + 2 * vertexDistance));
            _vertices.Add(new Vector3(x + 2 * vertexDistance, y + vertexDistance));
            _vertices.Add(new Vector3(x + vertexDistance, y + vertexDistance));

            // Triangle 7
            _vertices.Add(new Vector3(x + vertexDistance, y + 2 * vertexDistance));
            _vertices.Add(new Vector3(x + 2 * vertexDistance, y + 2 * vertexDistance));
            _vertices.Add(new Vector3(x + 2 * vertexDistance, y + vertexDistance));
        }


        private void GenerateTriangles() {
            _triangles.Clear();
            // The vertices are already arranged in the correct order (ie. triangle 0 has vertices indexed 0, 1, 2)
            _triangles = new List<int>();
            for (int i = 0; i < _vertices.Count; i++)
                _triangles.Add(i);
        }

        // Colors each triangle depending on its current state
        public void InitializeColors(SimulationMap<ExplorationCell> newMap) {
            int count = 0;
            foreach (var (index, explorationCell) in newMap) {
                var vertexIndex = index * 3;
                var color = SolidColor;
                if (explorationCell.IsExplorable)
                    color = explorationCell.IsExplored ? ExploredColor : StandardCellColor;
                _colors[vertexIndex] = color;
                _colors[vertexIndex + 1] = color;
                _colors[vertexIndex + 2] = color;
                if (!explorationCell.IsExplorable) count++;
            }
        }

        /// <summary>
        /// Updates the color of ALL triangles based on the given map and color function. This is an expensive operation
        /// and should be only called when it is necessary to replace all colors. To update a small subset of the
        /// triangles use the <see cref="UpdateColors"/> function.
        /// </summary>
        public void SetAllColors(SimulationMap<ExplorationCell> map, CellToColor cellToColor) {
            foreach (var (index, cell) in map) {
                var vertexIndex = index * 3;
                var color = cellToColor(cell);
                _colors[vertexIndex] = color;
                _colors[vertexIndex + 1] = color;
                _colors[vertexIndex + 2] = color;
            }

            mesh.colors32 = _colors;
        }
        
        /// <summary>
        /// Updates the color of ALL triangles based on the given map and color function. This is an expensive operation
        /// and should be only called when it is necessary to replace all colors. To update a small subset of the
        /// triangles use the <see cref="UpdateColors"/> function.
        /// </summary>
        public void SetAllColors(SimulationMap<ExplorationCell> map, CellIndexToColor cellToColor) {
            foreach (var (index, cell) in map) {
                var vertexIndex = index * 3;
                var color = cellToColor(index);
                _colors[vertexIndex] = color;
                _colors[vertexIndex + 1] = color;
                _colors[vertexIndex + 2] = color;
            }

            mesh.colors32 = _colors;
        }

        /// <summary>
        /// Updates the colors of the triangles corresponding to the given list of exploration cells.
        /// </summary>
        public void UpdateColors(IEnumerable<(int, ExplorationCell)> cellsWithIndices, CellToColor cellToColor) {
            foreach (var (index, cell) in cellsWithIndices) {
                var vertexIndex = index * 3;
                var color = cellToColor(cell);
                _colors[vertexIndex] = color;
                _colors[vertexIndex + 1] = color;
                _colors[vertexIndex + 2] = color;
            }

            mesh.colors32 = _colors;
        }
    }
}