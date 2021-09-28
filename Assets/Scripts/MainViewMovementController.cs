using System;
using System.Collections;
using System.Collections.Generic;
using Dora;
using UnityEngine;
using UnityEngine.Serialization;
using UnityEngine.UI;

[RequireComponent(typeof(Transform))]
public class MainViewMovementController : MonoBehaviour
{
    private Transform _mainViewTransform;
    
    [Range(0.01f, 2f)] public float movementSpeed; // Value set in inspector
    public List<UIMovementButton> buttons;

    private void Start()
    {
        _mainViewTransform = GetComponent<Transform>();
    }

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
            //_mainViewTransform.position += movementSpeed * button.Direction * Time.deltaTime;
        }
    }

}
