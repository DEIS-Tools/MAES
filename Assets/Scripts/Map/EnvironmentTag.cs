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

using System;
using Maes.Robot;
using Maes.Utilities;
using UnityEngine;
using YamlDotNet.RepresentationModel;

namespace Maes.Map {
    public class EnvironmentTag {
        
        public readonly int sender;
        private GameObject model;
        public Vector3 WorldPosition; // Coord in Unity
        public Vector2 MapPosition; // Coord in tile map
        public string Content;
        

        public EnvironmentTag(int sender, GameObject model, string content) {
            this.sender = sender;
            this.model = model;
            this.Content = content;
            this.WorldPosition = model.transform.position;
            this.MapPosition = WorldPosition;

            this.model.GetComponent<VisibleTagInfoHandler>().SetTag(this); 
        }

        public override string ToString() {
            return $"| Robot{sender} |:\n" + Content;
        }

        public void SetVisibility(bool val) {
            model.SetActive(val);
        }

        public string GetDebugInfo() {
            Vector2 position = GlobalSettings.IsRosMode ? Geometry.ToROSCoord(MapPosition) : MapPosition;
            
            return $"Tag content:  {Content}\n"
                   + $"Deposited by: Robot{sender}\n"
                   + $"Position:     ({position.x},{position.y})";
        }
    }
}