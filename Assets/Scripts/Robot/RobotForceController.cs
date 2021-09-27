using System;
using System.Collections;
using System.Collections.Generic;
using Dora.Robot;
using Dora.Robot.Task;
using JetBrains.Annotations;
using UnityEngine;

namespace Dora.Robot
{
    public class RobotForceController : MonoBehaviour, ISimulationUnit, IRobotController
    {
        public Transform leftWheel;
        public Transform rightWheel;

        private Rigidbody _rigidbody;
        private RobotStatus _currentStatus = RobotStatus.Idle;

        [Range(1, 150)] public int rotateForce = 60;
        [Range(1, 150)] public int moveForce = 80;

        private Vector3? _previousLeftWheelPosition = null;
        private Vector3? _previousRightWheelPosition = null;

        [CanBeNull] private ITask _currentTask;

        private void Start()
        {
            _rigidbody = GetComponent<Rigidbody>();
        }

        public void PhysicsUpdate(SimulationConfiguration config)
        {
            // Calculate movement delta between current and last physics tick
            var leftWheelVelocityVector = leftWheel.transform.position - _previousLeftWheelPosition ?? Vector3.zero;
            var rightWheelVelocityVector = rightWheel.transform.position - _previousRightWheelPosition ?? Vector3.zero;

            // For each wheel, determine whether it has moved forwards or backwards
            var forward = transform.forward;
            var leftWheelMoveDirection = Vector3.Dot(forward, leftWheelVelocityVector) < 0 ? -1f : 1f;
            var rightWheelMoveDirection = Vector3.Dot(forward, rightWheelVelocityVector) < 0 ? -1f : 1f;

            // Animate rotating wheels to match movement of the robot
            AnimateWheelRotation(leftWheel, leftWheelMoveDirection, leftWheelVelocityVector.magnitude);
            AnimateWheelRotation(rightWheel, rightWheelMoveDirection, rightWheelVelocityVector.magnitude);

            _previousLeftWheelPosition = leftWheel.position;
            _previousRightWheelPosition = rightWheel.position;

            // Update the current status to indicate whether the robot is currently moving
            if (rightWheelVelocityVector.magnitude > 0.01f || leftWheelVelocityVector.magnitude > 0.01f)
            {
                _currentStatus = RobotStatus.Moving;
            } else {
                _currentStatus = RobotStatus.Idle;
            }
            
            /*Debug.Log("Current velocity: " + leftWheelVelocityVector.magnitude + ", " + rightWheelVelocityVector.magnitude);
            
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
            }*/

            var directive = _currentTask?.GetNextDirective();
            if (directive != null)
                ApplyWheelForce(directive);
        }

        private void Update()
        {
            if (Input.GetButtonUp("Left"))
            {
                StartRotating(counterClockwise:true);
            }
            
            if (Input.GetButtonUp("Right"))
            {
                StartRotating(counterClockwise:false);
            }

            if (Input.GetButtonUp("Reverse"))
            {
                StopCurrentAction();
            }
        }

        
        public void LogicUpdate(SimulationConfiguration config)
        {
            // No logic to update
        }

        // Applies force at the positions of the wheels to create movement using physics simulation
        private void ApplyWheelForce(MovementDirective directive)
        {
            var forward = transform.forward;
            var force = moveForce;
            if (directive == MovementDirective.Left || directive == MovementDirective.Right) 
                force = rotateForce;
            _rigidbody.AddForceAtPosition(forward * force * directive.LeftWheelSpeed, leftWheel.position);
            _rigidbody.AddForceAtPosition(forward * force * directive.RightWheelSpeed, rightWheel.position);
        }

        // Rotates the given wheel depending on how far it has moved an in which direction
        private void AnimateWheelRotation(Transform wheel, float direction, float magnitude)
        {
            // This factor determines how forward movement of the wheel translates into rotation
            const float rotationFactor = 80f;
            wheel.Rotate(new Vector3(rotationFactor * direction * magnitude, 0f, 0f));
        }

        public object SaveState()
        {
            throw new NotImplementedException();
        }

        public void RestoreState(object stateInfo)
        {
            throw new NotImplementedException();
        }

        public RobotStatus GetStatus()
        {
            return _currentStatus;
        }

        public void Rotate(float degrees)
        {
            throw new NotImplementedException();
        }

        public void StartRotating(bool counterClockwise = false)
        {
            var currentStatus = GetStatus();
            if (currentStatus != RobotStatus.Idle) 
                throw new InvalidOperationException("Tried to start rotation action but current status is: " 
                                                    + Enum.GetName(typeof(RobotStatus), currentStatus)
                                                    + "Can only start a rotation action when current status is Idle");
            
            _currentTask = new InfiniteRotationTasK(counterClockwise);
        }
        
        public void StopCurrentAction()
        {
            _currentTask = null;
        }
    }
}