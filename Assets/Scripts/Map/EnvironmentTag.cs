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