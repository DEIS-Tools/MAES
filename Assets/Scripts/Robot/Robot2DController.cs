using System;
using System.Collections;
using System.Collections.Generic;
using System.Text;
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

        private const int RotateForce = 5;
        private const int MoveForce = 15;

        // Used for calculating wheel rotation for animation
        private Vector3? _previousLeftWheelPosition = null;
        private Vector3? _previousRightWheelPosition = null;

        private MonaRobot _robot;
        private RobotStatus _currentStatus = RobotStatus.Idle;
        [CanBeNull] private ITask _currentTask;
        
        public CommunicationManager CommunicationManager { get; set; }
        public SlamMap SlamMap { get; set; }
        
        // Whether the rigidbody is currently colliding with something
        private bool _isCurrentlyColliding = false;
        // Indicates whether the robot has entered a new collision since the previous logic update
        private bool _newCollisionSinceLastUpdate = false;

        // When the robot enters a collision (such as with a wall) the robot will only be notified of the
        // collision upon initial impact. If the robot continues to drive into the wall,
        // no further collision notifications will be received. To counteract this problem, the controller will 
        // reissue the collision notification if the collision flag is not cleared and the robot is following an
        // instruction to move forward. Because the acceleration of the robot is relatively slow, the collision
        // exit may not be triggered until after a few physics updates. This variable determines how many physics
        // updates to wait before re-declaring the collision.
        private readonly int _movementUpdatesBeforeRedeclaringCollision = 2;
        private int _physicsUpdatesSinceStartingMovement = 0;

        public Robot2DController(Rigidbody2D rigidbody, Transform transform, Transform leftWheel, Transform rightWheel, MonaRobot robot)
        {
            _rigidbody = rigidbody;
            _transform = transform;
            _leftWheel = leftWheel;
            _rightWheel = rightWheel;
            _robot = robot;
        }

        public int GetRobotID()
        {
            return _robot.id;
        }

        public void UpdateLogic()
        {
            // Clear the collision flag
            _newCollisionSinceLastUpdate = false;
        }

        public bool HasCollided()
        {
            return _newCollisionSinceLastUpdate;
        }
        
        public void NotifyCollided()
        {
            _newCollisionSinceLastUpdate = true;
            _isCurrentlyColliding = true;
            StopCurrentTask();
        }

        public void NotifyCollisionExit()
        {
            this._isCurrentlyColliding = false;
        }

        public object SaveState()
        {
            throw new System.NotImplementedException();
        }

        public void RestoreState(object stateInfo)
        {
            throw new System.NotImplementedException();
        }

        public void UpdateMotorPhysics()
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

            // Update the current status to indicate whether the robot is currently moving, stopping or idle
            if (_currentTask != null)
            {
                // The robot is currently following an assigned task
                _currentStatus = RobotStatus.Moving;
            }else if (rightWheelVelocityVector.magnitude > 0.01f || leftWheelVelocityVector.magnitude > 0.01f)
            {
                // The robot is moving but is not following a task, it assumed to be in the process of stopping
                _currentStatus = RobotStatus.Stopping;
            } else {
                _currentStatus = RobotStatus.Idle;
            }

            var isAttemptingToMoveForwards = _currentTask is MovementTask;
            if (_isCurrentlyColliding && isAttemptingToMoveForwards)
            {
                if (_physicsUpdatesSinceStartingMovement > _movementUpdatesBeforeRedeclaringCollision)
                    NotifyCollided();
                
                _physicsUpdatesSinceStartingMovement += 1;
            }
            else
            {
                // Reset counter
                _physicsUpdatesSinceStartingMovement = 0;
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

            // Force changes depending on whether the robot is rotating or accelerating
            var force = MoveForce;
            if (directive.IsRotational()) 
                force = RotateForce;
            
            // Apply force at teach wheel
            _rigidbody.AddForceAtPosition(forward * force * directive.LeftWheelSpeed, leftPosition);
            _rigidbody.AddForceAtPosition(forward * force * directive.RightWheelSpeed, rightPosition);
        }
        
        // Rotates the given wheel depending on how far it has moved an in which direction
        private void AnimateWheelRotation(Transform wheel, float direction, float magnitude)
        {
            // This factor determines how forward movement of the wheel translates into rotation
            const float rotationFactor = 180f;
            wheel.Rotate(new Vector3(rotationFactor * direction * magnitude, 0f, 0f));
        }
        
        public RobotStatus GetStatus()
        {
            if (_currentStatus == RobotStatus.Idle && _currentTask != null) return RobotStatus.Moving;
            return _currentStatus;
        }

        public void Rotate(float degrees)
        {
            if (_currentTask != null)
            {
                StopCurrentTask();
                return;
            }
            AssertRobotIsInIdleState("rotation");

            _currentTask = new FiniteRotationTask(_transform, degrees);
        }

        public void StartRotating(bool counterClockwise = false)
        {
            if (_currentTask != null)
            {
                StopCurrentTask();
                return;
            }
            var currentStatus = GetStatus();
            AssertRobotIsInIdleState("rotation");
            
            _currentTask = new InfiniteRotationTasK(counterClockwise);
        }
        
        
        public void StartMoving(bool reverse = false)
        {
            AssertRobotIsInIdleState("Moving Forwards");
            _currentTask = new MovementTask(reverse);
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
        
        
        public void StopCurrentTask()
        {
            _currentTask = null;
        }

        public void Broadcast(object data)
        {
            CommunicationManager.BroadcastMessage(_robot, data);
        }

        public List<object> ReceiveBroadcast()
        {
            return CommunicationManager.ReadMessages(_robot);
        }

        public string GetDebugInfo()
        {
            var info = new StringBuilder();
            info.AppendLine($"World Position: {_transform.position.x.ToString("#.0")}, {_transform.position.y.ToString("#.0")}");
            info.AppendLine($"Current task: {_currentTask?.GetType()}");
            return info.ToString();
        }

        public void Move(float distanceInMeters, bool reverse = false)
        {
            AssertRobotIsInIdleState($"Move forwards {distanceInMeters} meters");
            _currentTask = new FiniteMovementTask(_transform, distanceInMeters, reverse);
        }
    }
}
