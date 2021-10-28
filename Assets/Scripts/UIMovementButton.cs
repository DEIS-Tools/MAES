using UnityEngine;
using UnityEngine.EventSystems;
using UnityEngine.UI;

namespace Dora {
    [RequireComponent(typeof(Button))]
    public class UIMovementButton : MonoBehaviour, IPointerDownHandler, IPointerUpHandler {
        public enum Direction {
            Forwards,
            Backwards,
            Left,
            Right,
            In,
            Out,
            RLeft,
            RRight
        }

        public CameraController controller;
        public bool isActive;
        public Direction direction;

        private void Start() {
            isActive = false;
            controller.Subscribe(this);
        }

        public void OnPointerDown(PointerEventData eventData) {
            isActive = true;
        }

        public void OnPointerUp(PointerEventData eventData) {
            isActive = false;
        }
    }
}