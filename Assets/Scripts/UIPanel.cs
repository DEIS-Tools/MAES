using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Dora {
    [RequireComponent(typeof(RectTransform))]
    public class UIPanel : MonoBehaviour {
        public CameraController cameraController;

        void Start() {
            var t = GetComponent<RectTransform>();
            cameraController.Subscribe(t);
        }
    }
}