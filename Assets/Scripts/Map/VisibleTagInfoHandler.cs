using System;
using UnityEngine;
using YamlDotNet.Core.Tokens;

namespace Maes.Map {
    public class VisibleTagInfoHandler : MonoBehaviour {
        private EnvironmentTag _environmentTag;
        private delegate void OnVisibleTagSelectedDelegate(VisibleTagInfoHandler t);
        public Outline outline;
        private OnVisibleTagSelectedDelegate _onVisibleTagSelected;

        public void SetTag(EnvironmentTag t) {
            _environmentTag = t;
            _onVisibleTagSelected = Simulation.SingletonInstance.SetSelectedTag;
        }
        
        public void OnMouseEnter() {
            Tooltip.ShowTooltip_Static(_environmentTag.ToString());
        }

        public void OnMouseExit() {
            Tooltip.HideTooltip_Static();
        }

        public void OnMouseDown() {
            _onVisibleTagSelected(this);
        }

        public string GetDebugInfo() {
            return _environmentTag.GetDebugInfo();
        }
    }
}