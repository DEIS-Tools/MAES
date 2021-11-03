using Dora.MapGeneration;
using UnityEngine;
using static Dora.ExplorationAlgorithm.BrickAndMortar;

namespace Dora.ExplorationAlgorithm {
    public class BrickAndMortarTag: EnvironmentTaggingMap.ITag {

        public TileStatus Status;
        public readonly int ID;
        public const int UnknownNeighbour = -1;
        public int[] NeighbourIds = new int[8];

        public BrickAndMortarTag(TileStatus status, int id) {
            Status = status;
            ID = id;
            for (int i = 0; i < 8; i++) {
                NeighbourIds[i] = UnknownNeighbour;
            }
        }

        // Debug drawing
        private const float TagSquareSize = 0.3f;
        private readonly Vector3 _tagCubeSize = new Vector3(TagSquareSize, TagSquareSize, TagSquareSize);
        private readonly Vector3 _readableCubeSize = new Vector3(TagSquareSize + 0.01f, TagSquareSize + 0.01f, TagSquareSize + 0.01f);

        public void DrawGizmos(Vector3 position) {
            Gizmos.DrawCube(new Vector3(position.x, position.y, -_tagCubeSize.z / 2f), _tagCubeSize);
        }
        
        
    }
}