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
            
            //Debug.Log("Current velocity: " + leftWheelVelocityVector.magnitude + ", " + rightWheelVelocityVector.magnitude);
            
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

            
            
            if (Input.GetButton("Forward"))
            {
                directive = MovementDirective.Forward;
            }
            
            // Get directive from current task if present
            //var directive = _currentTask?.GetNextDirective();
            
            if (directive != null)
                ApplyWheelForce(directive);

            // Delete task once completed
            var isCurrentTaskCompleted = _currentTask?.IsCompleted() ?? false;
            if (isCurrentTaskCompleted)
            {
                _currentTask = null;
                Debug.Log(transform.rotation.eulerAngles.y);
            }
        }

        private void Update()
        {
            var shiftPressed = Input.GetKey(KeyCode.LeftShift);
            if (Input.GetButtonDown("Left"))
            {
                if (shiftPressed)
                    StartRotating(counterClockwise: true);
                else
                    Rotate(-20);

            }
            
            if (Input.GetButtonDown("Right"))
            {
                if (shiftPressed)
                    StartRotating(counterClockwise: false);
                else
                    Rotate(15);
            }

            if (Input.GetButtonDown("Reverse"))
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
            var leftPosition = leftWheel.position;
            var rightPosition = rightWheel.position;

            var forward = transform.forward;
            Debug.Log("Forward_: " + forward);
            var force = moveForce;
            if (directive == MovementDirective.Left || directive == MovementDirective.Right) 
                force = rotateForce;
            _rigidbody.AddForceAtPosition(forward * force * directive.LeftWheelSpeed, leftPosition);
            _rigidbody.AddForceAtPosition(forward * force * directive.RightWheelSpeed, rightPosition);
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
            AssertRobotIsInIdleState("rotation");

            _currentTask = new FiniteRotationTask(transform, degrees);
        }

        public void StartRotating(bool counterClockwise = false)
        {
            var currentStatus = GetStatus();
            AssertRobotIsInIdleState("rotation");
            
            _currentTask = new InfiniteRotationTasK(counterClockwise);
        }

        // Asserts that the current status is idle, and throws an exception if not
        private void AssertRobotIsInIdleState(String attemptedActionName)
        {
            var currentStatus = GetStatus();
            if (currentStatus != RobotStatus.Idle) 
                throw new InvalidOperationException("Tried to start action: '" + attemptedActionName 
                                                    + "' rotation action but current status is: " 
                                                    + Enum.GetName(typeof(RobotStatus), currentStatus)
                                                    + "Can only start '" + attemptedActionName 
                                                    + "' action when current status is Idle");
        }
        
        public void StopCurrentAction()
        {
            _currentTask = null;
        }
    }
}