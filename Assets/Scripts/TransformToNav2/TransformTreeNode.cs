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
using System.Runtime.CompilerServices;
using RosMessageTypes.Geometry;
using RosMessageTypes.Std;
using Unity.Robotics.Core;

using UnityEngine;

namespace Unity.Robotics.SlamExample
{
    class TransformTreeNode
    {
        public readonly GameObject SceneObject;
        public readonly List<TransformTreeNode> Children;
        public Transform Transform => SceneObject.transform;
        public string name => SceneObject.name;
        public bool IsALeafNode => Children.Count == 0;

        public TransformTreeNode(GameObject sceneObject)
        {
            SceneObject = sceneObject;
            Children = new List<TransformTreeNode>();
            PopulateChildNodes(this);

            // Debug.Log($"TransformTreeNode with name={name} has children={sceneObject.transform.childCount}");
        }

        public static TransformStampedMsg ToTransformStamped(TransformTreeNode node)
        {
            return node.Transform.ToROSTransformStamped(Clock.time);
        }

        static void PopulateChildNodes(TransformTreeNode tfNode) {
            var acceptedNames = new List<string>() {"base_footprint", "base_link", "base_scan"};
            var parentTransform = tfNode.Transform;
            for (var childIndex = 0; childIndex < parentTransform.childCount; ++childIndex) {
                var childTransform = parentTransform.GetChild(childIndex);
                var childGO = childTransform.gameObject;
                
                if (acceptedNames.Contains(childGO.name)) {
                    var childNode = new TransformTreeNode(childGO);
                    tfNode.Children.Add(childNode);
                }
            }
        }

        
    }
}