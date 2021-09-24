using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Serialization;
using UnityEngine.EventSystems;
using UnityEngine.UI;

[RequireComponent(typeof(Button))]
public class UIMovementButton : MonoBehaviour, IPointerDownHandler, IPointerUpHandler
{
    public MainViewMovementController controller;
    public Vector3 movement;
    public bool isActive;

    private void Start()
    {
        isActive = false;
        controller.Subscribe(this);
    }

    public void OnPointerDown(PointerEventData eventData)
    {
        isActive = true;
    }

    public void OnPointerUp(PointerEventData eventData)
    {
        isActive = false;
    }
}
