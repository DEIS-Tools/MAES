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