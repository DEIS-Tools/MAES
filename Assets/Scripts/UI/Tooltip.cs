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

using System;
using System.Collections;
using System.Collections.Generic;
using System.Runtime.CompilerServices;
using Maes.UI;
using UnityEngine;
using UnityEngine.UI;

namespace Maes {
    public class Tooltip : MonoBehaviour {
        private Text _text;
        private RectTransform _backgroundTransform;
        private static Tooltip _instance;

        private void Awake() {
            _instance = this;
            _backgroundTransform = transform.Find("Background").GetComponent<RectTransform>();
            _text = transform.Find("Text").GetComponent<Text>();
            HideTooltip();
        }

        private void ShowTooltip(string text) {
            gameObject.SetActive(true);
            _text.text = text;
            var padding = 2f;
            var bgSize = new Vector2(_text.preferredWidth + 2 * padding, _text.preferredHeight + 2 * padding);
            _backgroundTransform.sizeDelta = bgSize;
        }

        private void HideTooltip() {
            gameObject.SetActive(false);
        }

        private void Update() {
            // Have the tooltip follow the mouse-pointer around.
            RectTransformUtility.ScreenPointToLocalPointInRectangle(transform.parent.GetComponent<RectTransform>(), 
                Input.mousePosition, null, out var localPoint);
            transform.localPosition = localPoint + new Vector2(2f, 2f);
        }

        public static void ShowTooltip_Static(string text) {
            _instance.ShowTooltip(text);
        }

        public static void HideTooltip_Static() {
            _instance.HideTooltip();
        }
    }
}