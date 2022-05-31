// Copyright 2022 MAES
// 
// This file is part of MAES
// 
// MAES is free software: you can redistribute it and/or modify it under
// the terms of the GNU General Public License as published by the
// Free Software Foundation, either version 3 of the License, or (at your option)
// any later version.
// 
// MAES is distributed in the hope that it will be useful, but WITHOUT
// ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
// or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
// Public License for more details.
// 
// You should have received a copy of the GNU General Public License along
// with MAES. If not, see http://www.gnu.org/licenses/.
// 
// Contributors: Malte Z. Andreasen, Philip I. Holler and Magnus K. Jensen
// 
// Original repository: https://github.com/MalteZA/MAES

using System.Collections.Generic;
using Maes.Map;
using UnityEngine;

namespace Maes.ExplorationAlgorithm.BrickAndMortar {
    /*public class BrickAndMortarTag: EnvironmentTaggingMap.ITag {

        public BrickAndMortar.TileStatus Status;
        public readonly int ID;
        public const int UnknownNeighbour = -1;
        public int[] NeighbourIds = new int[8];

        public int LoopController = -1;

        private Dictionary<int, int> _robotIdToLastTraversalDirection = new Dictionary<int, int>();

        public BrickAndMortarTag(BrickAndMortar.TileStatus status, int id) {
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

        public void DrawTag(Vector3 position) {
            Gizmos.color = Status == BrickAndMortar.TileStatus.Visited ? _visitedColor : _exploredColor;
            Gizmos.DrawCube(new Vector3(position.x, position.y, -_tagCubeSize.z / 2f), _tagCubeSize);
        }
        
        
    }*/
}