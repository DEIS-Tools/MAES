using System.Collections.Generic;
using Dora.MapGeneration;
using UnityEngine;
using static Dora.ExplorationAlgorithm.BrickAndMortar;

namespace Dora.ExplorationAlgorithm {
    public class BrickAndMortarTag: EnvironmentTaggingMap.ITag {

        public TileStatus Status;
        public readonly int ID;
        public const int UnknownNeighbour = -1;
        public int[] NeighbourIds = new int[8];

        public int LoopController = -1;

        private Dictionary<int, int> _robotIdToLastTraversalDirection = new Dictionary<int, int>();

        public BrickAndMortarTag(TileStatus status, int id) {
            Status = status;
            ID = id;
            for (int i = 0; i < 8; i++) 
                NeighbourIds[i] = UnknownNeighbour;
        }

        // Returns the last direction that the given robot traveled when traversing through this tile
        public int? GetLastExitDirection(int robotID) {
            if (_robotIdToLastTraversalDirection.ContainsKey(robotID))
                return _robotIdToLastTraversalDirection[robotID];
            return null;
        }
        public void SetLastExitDirection(int robotID, int direction) {
            this._robotIdToLastTraversalDirection[robotID] = direction;
        }

        // Performs a loop cleaning action on the for the given robot
        public void Clean(int robotID) {
            _robotIdToLastTraversalDirection.Remove(robotID);
            if (LoopController == robotID) LoopController = -1;
        }

        // Debug drawing
        private const float TagSquareSize = 0.3f;
        private readonly Vector3 _tagCubeSize = new Vector3(TagSquareSize, TagSquareSize, TagSquareSize);

        private Color _exploredColor = new Color(50f / 255f, 200f / 255f, 180f / 255f, 1f);
        private Color _visitedColor = new Color(50f / 255f, 50f / 255f, 50f / 255f, 1f);

        public void DrawGizmos(Vector3 position) {
            Gizmos.color = Status == TileStatus.Visited ? _visitedColor : _exploredColor;
            Gizmos.DrawCube(new Vector3(position.x, position.y, -_tagCubeSize.z / 2f), _tagCubeSize);
        }
        
        
    }
}