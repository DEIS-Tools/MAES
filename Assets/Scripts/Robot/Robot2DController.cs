using System;
using System.Collections;
using System.Collections.Generic;
using Dora.Robot;
using Dora.Robot.Task;
using JetBrains.Annotations;
using UnityEngine;

namespace Dora
{
    public class Robot2DController : MonoBehaviour, ISimulationUnit, IRobotController
    {

        private Rigidbody2D _rigidbody;
        public Transform leftWheel;
        public Transform rightWheel;

        [Range(1, 150)] public int rotateForce = 60;
        [Range(1, 150)] public int moveForce = 80;
        
        // Used for calculating wheel rotation for animation
        private Vector3? _previousLeftWheelPosition = null;
        private Vector3? _previousRightWheelPosition = null;
        
        private RobotStatus _currentStatus = RobotStatus.Idle;
        
        [CanBeNull] private ITask _currentTask;
        
        void Start()
        {
            //Physics2D.gravity = Vector2.zero;
            _rigidbody = GetComponent<Rigidbody2D>();
        }

        
        
        public object SaveState()
        {
            
            throw new System.NotImplementedException();
        }

        public void RestoreState(object stateInfo)
        {
            throw new System.NotImplementedException();
        }

        public void LogicUpdate(SimulationConfiguration config)
        {
            // No logic
        }

        private void Update()
        {
            var shiftPressed = Input.GetKey(KeyCode.LeftShift);
            if (Input.GetButtonDown("Left"))
            {
                if (shiftPressed)
                    StartRotating(counterClockwise: true);
                else
                    Rotate(-90);
            }
            
            if (Input.GetButtonDown("Right"))
            {
                if (shiftPressed)
                    StartRotating(counterClockwise: false);
                else
                    Rotate(90);
            }

            if (Input.GetButtonDown("Reverse"))
            {
                MoveBackwards();
            }

            if (Input.GetButtonDown("Forward"))
            {
                MoveForward();
            }
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

            // Get directive from current task if present
            var directive = _currentTask?.GetNextDirective();
            
            if (directive != null)
                ApplyWheelForce(directive);

            // Delete task once completed
            var isCurrentTaskCompleted = _currentTask?.IsCompleted() ?? false;
            if (isCurrentTaskCompleted)
            {
                _currentTask = null;
            }

            if (directive != null)
                ApplyWheelForce(directive);
        }
        
        // Applies force at the positions of the wheels to create movement using physics simulation
        private void ApplyWheelForce(MovementDirective directive)
        {
            var leftPosition = leftWheel.position;
            var rightPosition = rightWheel.position;

            var forward = transform.up;

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
        
        public RobotStatus GetStatus()
        {
            return _currentStatus;
        }

        public void Rotate(float degrees)
        {
            if (_currentTask != null)
            {
                StopCurrentAction();
                return;
            }
            AssertRobotIsInIdleState("rotation");

            _currentTask = new FiniteRotationTask(transform, degrees);
        }

        public void StartRotating(bool counterClockwise = false)
        {
            if (_currentTask != null)
            {
                StopCurrentAction();
                return;
            }
            var currentStatus = GetStatus();
            AssertRobotIsInIdleState("rotation");
            
            _currentTask = new InfiniteRotationTasK(counterClockwise);
        }
        
        
        public void MoveForward()
        {
            if (_currentTask != null)
            {
                StopCurrentAction();
                return;
            }
            AssertRobotIsInIdleState("Moving Forwards");
            _currentTask = new MovementTask();
        }

        public void MoveBackwards()
        {
            if (_currentTask != null)
            {
                StopCurrentAction();
                return;
            }
            AssertRobotIsInIdleState("Moving Forwards");
            _currentTask = new MovementTask(reverse:true);
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
