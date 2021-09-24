using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Serialization;
using UnityEngine.EventSystems;

public class UIMovementButton : MonoBehaviour, IPointerClickHandler
{
    public MainViewMovementController controller;
    public Vector3 movement = new Vector3();

    private void Start()
    {
        controller.Subscribe(this);
    }

    public void OnPointerClick(PointerEventData eventData)
    {
        controller.OnButtonPressed(this);
    }
}
