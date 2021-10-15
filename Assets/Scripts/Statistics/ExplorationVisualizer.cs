using System.Collections.Generic;
using UnityEngine;

namespace Dora.Statistics
{
    public class ExplorationVisualizer
    {

        private readonly int _widthInTiles, _heightInTiles;
        private readonly int _widthInVertices, _heightInVertices;
        private readonly float _scale;
        private readonly Vector3 _offset;

        private List<Vector3> _vertices = new List<Vector3>();
        private List<int> _triangles = new List<int>();

        private const int ResolutionMultiplier = 2;
        
        public ExplorationVisualizer(int widthInTiles, int heightInTiles, float scale, Vector3 offset)
        {
            _widthInTiles = widthInTiles;
            _heightInTiles = heightInTiles;
            _scale = scale;
            _offset = offset;
            _widthInVertices = widthInTiles * ResolutionMultiplier;
            _heightInVertices = heightInTiles * ResolutionMultiplier;
            GenerateVertices();
        }

        private void GenerateVertices()
        {
            float coordinateMultiplier = _scale / ResolutionMultiplier;
            for (int y = 0; y < _heightInVertices; y++)
            {
                for (int x = 0; x < _widthInVertices; x++)
                {
                    _vertices.Add(new Vector3(x * coordinateMultiplier, y * coordinateMultiplier, 0f));
                }                
            }
        }
    }
}