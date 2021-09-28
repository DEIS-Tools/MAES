using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace Dora
{
    public class CameraController : MonoBehaviour
    {
        public Camera cam;
        public Transform cameraTransform;

        public float movementSpeed;
        public float movementTime;
        public float rotationAmount;
        public Vector3 zoomAmount;

        public Vector3 newPosition;
        public Quaternion newRotation;
        public Vector3 newZoom;

        public Vector3 dragStartPosition;
        public Vector3 dragCurrentPosition;

        public Vector3 rotateStartPosition;
        public Vector3 rotateCurrentPosition;

        public List<UIMovementButton> buttons;

        // Start is called before the first frame update
        void Start()
        {
            var t = transform; // Temp storage of build-in is (apparently) more efficient than repeated access.
            newPosition = t.position;
            newRotation = t.rotation;
            newZoom = cameraTransform.localPosition;
        }

        // Update is called once per frame
        void Update()
        {
            HandleMouseInput();
            HandleMovementInput();
        }

        public void Subscribe(UIMovementButton button)
        {
            if (buttons == null)
            {
                buttons = new List<UIMovementButton>();
            }

            buttons.Add(button);
            buttons = buttons.OrderBy(b => b.direction).ToList();
        }

        void HandleMouseInput()
        {
            if (Input.mouseScrollDelta.y != 0)
            {
                newZoom += Input.mouseScrollDelta.y * zoomAmount;
            }

            if (Input.GetMouseButtonDown(0))
            {
                var plane = new Plane(Vector3.up, Vector3.zero);
                var ray = cam.ScreenPointToRay(Input.mousePosition);

                if (plane.Raycast(ray, out var entry))
                {
                    dragStartPosition = ray.GetPoint(entry);
                }
            }

            if (Input.GetMouseButton(0))
            {
                var plane = new Plane(Vector3.up, Vector3.zero);
                var ray = cam.ScreenPointToRay(Input.mousePosition);

                if (!plane.Raycast(ray, out var entry)) return;
                dragCurrentPosition = ray.GetPoint(entry);

                newPosition = transform.position + (dragStartPosition - dragCurrentPosition);
            }

            if (Input.GetMouseButtonDown(1))
            {
                rotateStartPosition = Input.mousePosition;
            }

            if (!Input.GetMouseButton(1)) return;
            rotateCurrentPosition = Input.mousePosition;

            var diff = rotateStartPosition - rotateCurrentPosition;

            rotateStartPosition = rotateCurrentPosition;

            newRotation *= Quaternion.Euler(Vector3.up * (-1 * diff.x / 5f));
        }

        void HandleMovementInput()
        {
            var t = transform;
            if (Input.GetKey(KeyCode.I) || buttons[(int) UIMovementButton.Direction.Forwards].isActive)
            {
                newPosition += t.forward * movementSpeed;
            }

            if (Input.GetKey(KeyCode.K) || buttons[(int) UIMovementButton.Direction.Backwards].isActive)
            {
                newPosition += t.forward * -1 * movementSpeed;
            }

            if (Input.GetKey(KeyCode.J) || buttons[(int) UIMovementButton.Direction.Left].isActive)
            {
                newPosition += t.right * -1 * movementSpeed;
            }

            if (Input.GetKey(KeyCode.L) || buttons[(int) UIMovementButton.Direction.Right].isActive)
            {
                newPosition += t.right * movementSpeed;
            }

            if (Input.GetKey(KeyCode.U) || buttons[(int) UIMovementButton.Direction.RLeft].isActive)
            {
                newRotation *= Quaternion.Euler(Vector3.up * rotationAmount);
            }

            if (Input.GetKey(KeyCode.O) || buttons[(int) UIMovementButton.Direction.RRight].isActive)
            {
                newRotation *= Quaternion.Euler(Vector3.up * -1 * rotationAmount);
            }

            if (Input.GetKey(KeyCode.Period) || Input.GetKey(KeyCode.Plus) || Input.GetKey(KeyCode.KeypadPlus) ||
                buttons[(int) UIMovementButton.Direction.In].isActive)
            {
                newZoom += zoomAmount;
            }

            if (Input.GetKey(KeyCode.Comma) || Input.GetKey(KeyCode.Minus) || Input.GetKey(KeyCode.KeypadMinus) ||
                buttons[(int) UIMovementButton.Direction.Out].isActive)
            {
                newZoom -= zoomAmount;
            }

            t.position = Vector3.Lerp(t.position, newPosition, Time.deltaTime * movementTime);
            t.rotation = Quaternion.Lerp(t.rotation, newRotation, Time.deltaTime * movementTime);
            cameraTransform.localPosition =
                Vector3.Lerp(cameraTransform.localPosition, newZoom, Time.deltaTime * movementTime);
        }
    }
}