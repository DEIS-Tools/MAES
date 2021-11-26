using Maes.Map;
using UnityEngine;

namespace Maes.ExplorationAlgorithm.TheNextFrontier {
    public class TnfTag : EnvironmentTaggingMap.ITag {
        public readonly int ID;
        private Color _color;

        public TnfTag(int id, bool movementTarget) {
            ID = id;
            _color = movementTarget ? Color.red : Color.yellow;
        }

        public TnfTag(int id, Color customColor) {
            ID = id;
            _color = customColor;
        }


        // Debug drawing
        private const float TagSquareSize = 0.3f;
        private readonly Vector3 _tagCubeSize = new Vector3(TagSquareSize, TagSquareSize, TagSquareSize);

        public void DrawGizmos(Vector3 position) {
            Gizmos.color = _color;
            Gizmos.DrawCube(new Vector3(position.x, position.y, -_tagCubeSize.z / 2f), _tagCubeSize);
        }
    }
}