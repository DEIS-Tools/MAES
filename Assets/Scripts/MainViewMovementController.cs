using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Serialization;
using UnityEngine.UI;

public class MainViewMovementController : MonoBehaviour
{
    [Range(0.01f, 2f)]
    public float movementSpeed;
    public Transform mainViewTransform;
    public List<UIMovementButton> buttons;

    public void Subscribe(UIMovementButton button)
    {
        buttons ??= new List<UIMovementButton>();
        buttons.Add(button);
    }

    public void Update()
    {
        foreach (var button in buttons)
        {
            if (!button.isActive) continue;
            mainViewTransform.position += movementSpeed * button.movement * Time.deltaTime;
        }
    }

}
