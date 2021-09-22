using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class RobotController : MonoBehaviour, IRobotController
{
    public float motorForce = 1000f;

    public WheelCollider leftWheelCollider, rightWheelCollider;
    public Transform leftWheelTrans, rightWheelTrans;
    
    
    void FixedUpdate()
    {
        leftWheelCollider.motorTorque = 0f;
        rightWheelCollider.motorTorque = 0f;
        
        if (Input.GetButton("Left"))
        {
            RotateCounterClockwise();
        }
        else if (Input.GetButton("Right"))
        {
            RotateClockwise();
        }

        else if (Input.GetButton("Forward"))
        {
            MoveForward();
        }
        else if (Input.GetButton("Reverse"))
        {
            MoveBackwards();
        }
        else
        {
            leftWheelCollider.motorTorque = 0f;
            rightWheelCollider.motorTorque = 0f;
        }
    }

    public void SenseSurroundings()
    {
        Debug.Log("Sensing surroundings!");
    }
    
    public void RotateCounterClockwise()
    {
        leftWheelCollider.motorTorque = -motorForce;
        rightWheelCollider.motorTorque = motorForce;
        RotateRightWheelForward();
        RotateLeftWheelBackwards();
        
    }

    public void RotateClockwise()
    {
        leftWheelCollider.motorTorque = motorForce;
        rightWheelCollider.motorTorque = -motorForce;
        RotateRightWheelBackwards();
        RotateLeftWheelForward();
    }

    public void MoveForward()
    {
        leftWheelCollider.motorTorque = motorForce;
        rightWheelCollider.motorTorque = motorForce;
        RotateRightWheelForward();
        RotateLeftWheelForward();
    }

    public void MoveBackwards()
    {
        leftWheelCollider.motorTorque = -motorForce;
        rightWheelCollider.motorTorque = -motorForce;
        RotateLeftWheelBackwards();
        RotateRightWheelBackwards();
    }

    private void RotateLeftWheelForward()
    {
        leftWheelTrans.Rotate(new Vector3(100f * Time.deltaTime, 0f, 0f));
    }

    private void RotateRightWheelForward()
    {
        rightWheelTrans.Rotate(new Vector3(100f * Time.deltaTime, 0f, 0f));
    }

    private void RotateLeftWheelBackwards()
    {
        leftWheelTrans.Rotate(new Vector3(-100f * Time.deltaTime, 0f, 0f));
    }

    private void RotateRightWheelBackwards()
    {
        rightWheelTrans.Rotate(new Vector3(-100f * Time.deltaTime, 0f, 0f));
    }
}
