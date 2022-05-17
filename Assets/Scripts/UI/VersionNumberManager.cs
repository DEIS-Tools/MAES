using System;
using UnityEditor;
using UnityEngine;
using UnityEngine.UI;

namespace Maes.UI {
    public class VersionNumberManager: MonoBehaviour {
        
        private void Start() {
            UpdateVersionNumberText();
        }

        private void UpdateVersionNumberText() {
            var versionNumberText = this.gameObject.GetComponent<Text>();
            versionNumberText.text = "v." + Application.version;
        }
    }
}