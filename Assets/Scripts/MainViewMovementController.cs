using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Serialization;
using UnityEngine.UI;

public class MainViewMovementController : MonoBehaviour
{
    public Transform mainViewTransform;
    public List<UIMovementButton> buttons;

    public void Subscribe(UIMovementButton button)
    {
        buttons ??= new List<UIMovementButton>();
        
        buttons.Add(button);
    }

    public void OnButtonPressed(UIMovementButton button)
    {
        mainViewTransform.Translate(button.movement);
    }
}
