using UnityEngine;

namespace Maes.UI {
    [RequireComponent(typeof(RectTransform))]
    public class UIPanel : MonoBehaviour {
        public CameraController cameraController;

        void Start() {
            var t = GetComponent<RectTransform>();
            cameraController.Subscribe(t);
        }
    }
}