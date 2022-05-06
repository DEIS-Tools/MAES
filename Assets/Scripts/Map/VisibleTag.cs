using Maes.Robot;
using UnityEngine;
using YamlDotNet.RepresentationModel;

namespace Maes.Map {
    public class VisibleTag : EnvironmentTaggingMap.ITag {

        public EnvironmentTaggingMap.ITag innerITag;
        public readonly int sender;
        private GameObject model;
        

        public VisibleTag(int sender, GameObject model, EnvironmentTaggingMap.ITag innerITag) {
            this.sender = sender;
            this.innerITag = innerITag;
            this.model = model;
            this.model.GetComponent<VisibleTagInfoHandler>().SetTag(this); 
        }
        
        public void DrawTag(Vector3 position) {
            model.transform.position = position;
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
                 + $"Position:     {model.transform.position}";
        }

        public void Unselect() {
            throw new System.NotImplementedException();
        }

        public void Select() {
            
        }
    }
}