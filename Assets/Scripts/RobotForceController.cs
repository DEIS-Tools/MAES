using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Dora
{
    public class RobotForceController : MonoBehaviour, SimulationUnit
    {
        
        public Transform leftWheel;
        public Transform rightWheel;

        [Range(1, 100)]
        public int rorateForce = 10;
        
        [Range(1, 100)]
        public int moveForce = 10;

        private Vector3? lastLeftWheelPosition = null;
        private Vector3? lastRightWheelPosition = null;
        
        public void SimUpdate(SimulationConfiguration config)
        {
            if (Input.GetButton("Left"))
            {
                GetComponent<Rigidbody>().AddForceAtPosition(transform.forward * rorateForce, leftWheel.position);
                GetComponent<Rigidbody>().AddForceAtPosition(transform.forward * -rorateForce, rightWheel.position);
            }

            if (Input.GetButton("Right"))
            {
                GetComponent<Rigidbody>().AddForceAtPosition(transform.forward * -rorateForce, leftWheel.position);
                GetComponent<Rigidbody>().AddForceAtPosition(transform.forward * rorateForce, rightWheel.position);
            }
            
            if (Input.GetButton("Forward"))
            {
                GetComponent<Rigidbody>().AddForceAtPosition(transform.forward * moveForce, leftWheel.position);
                GetComponent<Rigidbody>().AddForceAtPosition(transform.forward * moveForce, rightWheel.position);
            }
            
            if (Input.GetButton("Reverse"))
            {
                GetComponent<Rigidbody>().AddForceAtPosition(transform.forward * -moveForce, leftWheel.position);
                GetComponent<Rigidbody>().AddForceAtPosition(transform.forward * -moveForce, rightWheel.position);
            }
            
            var leftWheelDistance = (leftWheel.transform.position - lastLeftWheelPosition)?.magnitude;
            var rightWheelDistance = (rightWheel.transform.position - lastRightWheelPosition)?.magnitude;

            Debug.Log("Distance: " + lastRightWheelPosition.ToString() ?? "");
            
            // TODO: Rotation direction
            leftWheel.Rotate(new Vector3(100f * leftWheelDistance ?? 0f, 0f, 0f));
            rightWheel.Rotate(new Vector3(80f * rightWheelDistance ?? 0f, 0f, 0f));

            lastLeftWheelPosition = leftWheel.position;
            lastRightWheelPosition = rightWheel.position;
        }
        
    }
}
