using Dora.MapGeneration;
using UnityEngine;

namespace Dora.ExplorationAlgorithm {
    public class BrickAndMortarTag: EnvironmentTaggingMap.ITag {
        
        // Debug drawing
        private const float TagSquareSize = 0.3f;
        private readonly Vector3 _tagCubeSize = new Vector3(TagSquareSize, TagSquareSize, TagSquareSize);
        private readonly Vector3 _readableCubeSize = new Vector3(TagSquareSize + 0.01f, TagSquareSize + 0.01f, TagSquareSize + 0.01f);
        
        public void DrawGizmos(Vector3 position) {
            Gizmos.DrawCube(new Vector3(position.x, position.y, -_tagCubeSize.z / 2f), _tagCubeSize);
        }
        
        
    }
}