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

        public void DrawTag(Vector3 position) {
            Gizmos.color = _color;
            Gizmos.DrawCube(new Vector3(position.x, position.y, -_tagCubeSize.z / 2f), _tagCubeSize);
        }
    }
}