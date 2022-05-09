using Maes.Robot;
using UnityEngine;
using YamlDotNet.RepresentationModel;

namespace Maes.Map {
    public class VisibleTag : EnvironmentTaggingMap.ITag {

        public EnvironmentTaggingMap.ITag innerITag;
        public readonly int sender;
        private GameObject model;
        private Quaternion _initialRotation;
        private Vector3 _position;
        

        public VisibleTag(int sender, GameObject model, EnvironmentTaggingMap.ITag innerITag) {
            this.sender = sender;
            this.innerITag = innerITag;
            this.model = model;
            this._position = model.transform.position;
            this._initialRotation = model.transform.rotation;
            
            this.model.GetComponent<VisibleTagInfoHandler>().SetTag(this); 
        }
        
        public void DrawTag(Vector3 position) {
            model.transform.position = position;
            model.transform.rotation = _initialRotation;
            // innerITag.DrawGizmos(position);
        }

        public override string ToString() {
            return $"| Robot{sender} |:\n" + innerITag;
        }

        public void SetVisibility(bool val) {
            model.SetActive(val);
        }

        public string GetDebugInfo() {
            return $"Tag content:  {innerITag}\n"
                   + $"Deposited by: Robot{sender}\n"
                   + $"Position:     ({this._position.x},{this._position.y})";
        }

        public void Unselect() {
            throw new System.NotImplementedException();
        }

        public void Select() {
            
        }
    }
}