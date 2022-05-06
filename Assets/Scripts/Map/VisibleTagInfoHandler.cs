using System;
using UnityEngine;
using YamlDotNet.Core.Tokens;

namespace Maes.Map {
    public class VisibleTagInfoHandler : MonoBehaviour {
        private VisibleTag _visibleTag;
        private delegate void OnVisibleTagSelectedDelegate(VisibleTagInfoHandler t);
        public Outline outline;
        private OnVisibleTagSelectedDelegate _onVisibleTagSelected;

        public void SetTag(VisibleTag t) {
            _visibleTag = t;
            _onVisibleTagSelected = Simulation.SingletonInstance.SetSelectedTag;
        }
        
        public void OnMouseEnter() {
            Tooltip.ShowTooltip_Static(_visibleTag.ToString());
        }

        public void OnMouseExit() {
            Tooltip.HideTooltip_Static();
        }

        public void OnMouseDown() {
            _onVisibleTagSelected(this);
        }

        public string GetDebugInfo() {
            return _visibleTag.GetDebugInfo();
        }
    }
}