// Copyright 2022 MAES
// 
// This file is part of MAES
// 
// MAES is free software: you can redistribute it and/or modify it under
// the terms of the GNU General Public License as published by the
// Free Software Foundation, either version 3 of the License, or (at your option)
// any later version.
// 
// MAES is distributed in the hope that it will be useful, but WITHOUT
// ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
// or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
// Public License for more details.
// 
// You should have received a copy of the GNU General Public License along
// with MAES. If not, see http://www.gnu.org/licenses/.
// 
// Contributors: Malte Z. Andreasen, Philip I. Holler and Magnus K. Jensen
// 
// Original repository: https://github.com/MalteZA/MAES

using UnityEngine;
using UnityEngine.EventSystems;
using UnityEngine.UI;

namespace Maes.UI {
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