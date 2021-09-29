using System;
using System.Collections;
using System.Collections.Generic;
using Dora.Robot;
using Dora.Robot.Task;
using JetBrains.Annotations;
using UnityEngine;

namespace Dora
{
    public class Robot2DController : IRobotController
    {

        private Rigidbody2D _rigidbody;
        private Transform _transform;
        private Transform _leftWheel;
        private Transform _rightWheel;

        private const int RotateForce = 20;
        private const int MoveForce = 30;

        // Used for calculating wheel rotation for animation
        private Vector3? _previousLeftWheelPosition = null;
        private Vector3? _previousRightWheelPosition = null;

        private RobotStatus _currentStatus = RobotStatus.Idle;
        [CanBeNull] private ITask _currentTask;

        public Robot2DController(Rigidbody2D rigidbody, Transform transform, Transform leftWheel, Transform rightWheel)
        {
            _rigidbody = rigidbody;
            _transform = transform;
            _leftWheel = leftWheel;
            _rightWheel = rightWheel;
        }
        
        public object SaveState()
        {
            
            throw new System.NotImplementedException();
        }

        public void RestoreState(object stateInfo)
        {
            throw new System.NotImplementedException();
        }

        public void UpdateMotorPhysics(SimulationConfiguration config)
        {
            // Calculate movement delta between current and last physics tick
            var leftWheelVelocityVector = _leftWheel.transform.position - _previousLeftWheelPosition ?? Vector3.zero;
            var rightWheelVelocityVector = _rightWheel.transform.position - _previousRightWheelPosition ?? Vector3.zero;

            // For each wheel, determine whether it has moved forwards or backwards
            var forward = _transform.forward;
            var leftWheelMoveDirection = Vector3.Dot(forward, leftWheelVelocityVector) < 0 ? -1f : 1f;
            var rightWheelMoveDirection = Vector3.Dot(forward, rightWheelVelocityVector) < 0 ? -1f : 1f;

            // Animate rotating wheels to match movement of the robot
            AnimateWheelRotation(_leftWheel, leftWheelMoveDirection, leftWheelVelocityVector.magnitude);
            AnimateWheelRotation(_rightWheel, rightWheelMoveDirection, rightWheelVelocityVector.magnitude);

            _previousLeftWheelPosition = _leftWheel.position;
            _previousRightWheelPosition = _rightWheel.position;

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
            var leftPosition = _leftWheel.position;
            var rightPosition = _rightWheel.position;

            var forward = _transform.up;

            var force = MoveForce;
            if (directive.IsRotational()) 
                force = RotateForce;
            
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

            _currentTask = new FiniteRotationTask(_transform, degrees);
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
