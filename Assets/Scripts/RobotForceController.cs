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

        [Range(1, 150)] public int rorateForce = 70;

        [Range(1, 150)] public int moveForce = 95;

        private Vector3? lastLeftWheelPosition = null;
        private Vector3? lastRightWheelPosition = null;

        public void SimUpdate(SimulationConfiguration config)
        {
            // Calculate movement between current and last physics tick
            var leftWheelDifferenceVector = leftWheel.transform.position - lastLeftWheelPosition ?? Vector3.zero;
            var rightWheelDifferenceVector = rightWheel.transform.position - lastRightWheelPosition ?? Vector3.zero;

            // For each wheel, determine whether it has moved forwards or backwards
            var forward = transform.forward;
            var leftWheelMoveDirection = Vector3.Dot(forward, leftWheelDifferenceVector) < 0 ? -1f : 1f;
            var rightWheelMoveDirection = Vector3.Dot(forward, rightWheelDifferenceVector) < 0 ? -1f : 1f;

            // Animate rotating wheels to match movement of the robot
            leftWheel.Rotate(new Vector3(80f * leftWheelMoveDirection* leftWheelDifferenceVector.magnitude, 0f, 0f));
            rightWheel.Rotate(new Vector3(80f * rightWheelMoveDirection * rightWheelDifferenceVector.magnitude, 0f, 0f));

            lastLeftWheelPosition = leftWheel.position;
            lastRightWheelPosition = rightWheel.position;
            
            if (Input.GetButton("Left"))
            {
                GetComponent<Rigidbody>().AddForceAtPosition(transform.forward * -rorateForce, leftWheel.position);
                GetComponent<Rigidbody>().AddForceAtPosition(transform.forward * rorateForce, rightWheel.position);
            }

            if (Input.GetButton("Right"))
            {
                GetComponent<Rigidbody>().AddForceAtPosition(transform.forward * rorateForce, leftWheel.position);
                GetComponent<Rigidbody>().AddForceAtPosition(transform.forward * -rorateForce, rightWheel.position);
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

            
        }
    }
}