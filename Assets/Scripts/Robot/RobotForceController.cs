using System;
using System.Collections;
using System.Collections.Generic;
using Dora.Robot;
using UnityEngine;

namespace Dora.Robot
{
    public class RobotForceController : MonoBehaviour, SimulationUnit
    {
        public Transform leftWheel;
        public Transform rightWheel;

        private Rigidbody _rigidbody;

        [Range(1, 150)] public int rotateForce = 70;

        [Range(1, 150)] public int moveForce = 95;

        private Vector3? lastLeftWheelPosition = null;
        private Vector3? lastRightWheelPosition = null;

        private void Start()
        {
            _rigidbody = GetComponent<Rigidbody>();
        }

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
            AnimateWheelRotation(leftWheel, leftWheelMoveDirection, leftWheelDifferenceVector.magnitude);
            AnimateWheelRotation(rightWheel, rightWheelMoveDirection, rightWheelDifferenceVector.magnitude);

            lastLeftWheelPosition = leftWheel.position;
            lastRightWheelPosition = rightWheel.position;
            
            MovementDirective directive = MovementDirective.NoMovement;
            
            if (Input.GetButton("Left"))
            {
                directive = MovementDirective.Left;
            }

            if (Input.GetButton("Right"))
            {
                directive = MovementDirective.Right;
            }

            if (Input.GetButton("Forward"))
            {
                directive = MovementDirective.Forward;
            }

            if (Input.GetButton("Reverse"))
            {
                directive = MovementDirective.Reverse;
            }
            
            ApplyWheelForce(directive);
        }

        // Applies force at the positions of the wheels to create movement using physics simulation
        private void ApplyWheelForce(MovementDirective directive)
        {
            var forward = transform.forward;
            _rigidbody.AddForceAtPosition(forward * moveForce * directive.LeftWheelSpeed, leftWheel.position);
            _rigidbody.AddForceAtPosition(forward * moveForce * directive.RightWheelSpeed, rightWheel.position);
        }

        // Rotates the given wheel depending on how far it has moved an in which direction
        private void AnimateWheelRotation(Transform wheel, float direction, float magnitude)
        {
            // This factor determines how forward movement of the wheel translates into rotation
            const float rotationFactor = 80f;
            wheel.Rotate(new Vector3(rotationFactor * direction * magnitude, 0f, 0f));
        }
    }
}