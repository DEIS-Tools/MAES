// Copyright 2022 MAES
// 
// This file is part of MAES
// 
// MAES is free software: you can redistribute it and/or modify it under
// the terms of the GNU General Public License as published by the
// Free Software Foundation, either version 3 of the License, or (at your option)
// any later version.
// 
// MAES is distributed in the hope that it will be useful, but WITHOUT
// ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
// or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
// Public License for more details.
// 
// You should have received a copy of the GNU General Public License along
// with MAES. If not, see http://www.gnu.org/licenses/.
// 
// Contributors: Malte Z. Andreasen, Philip I. Holler and Magnus K. Jensen
// 
// Original repository: https://github.com/MalteZA/MAES

using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Maes.Map;
using Maes.Robot.Task;
using Maes.Utilities;
using UnityEngine;

namespace Maes.Robot
{
    public class Robot2DController : IRobotController
    {
        private Rigidbody2D _rigidbody;
        public Transform Transform { get; private set; }
        public Transform LeftWheel { get; private set; }
        public Transform RightWheel { get; private set; }

        private const int RotateForce = 5;
        private const int MoveForce = 15;

        // Used for calculating wheel rotation for animation
        private Vector3? _previousLeftWheelPosition = null;
        private Vector3? _previousRightWheelPosition = null;

        private MonaRobot _robot;
        private RobotStatus _currentStatus = RobotStatus.Idle;
        private ITask? CurrentTask;

        internal CommunicationManager CommunicationManager { get; set; }
        public SlamMap SlamMap { get; set; }
        private Queue<Vector2Int> _currentPath = new();
        private Vector2Int _currentTarget = new();

        // Returns the counterclockwise angle in degrees between the forward orientation of the robot and the x-axis
        public float GetForwardAngleRelativeToXAxis()
        {
            return ((Vector2)Transform.up).GetAngleRelativeToX();
        }

        private Vector2 GetRobotDirectionVector()
        {
            var angle = GetForwardAngleRelativeToXAxis();
            return new Vector2(Mathf.Cos(angle * Mathf.Deg2Rad), Mathf.Sin(angle * Mathf.Deg2Rad));
        }

        // Whether the rigidbody is currently colliding with something
        private bool _isCurrentlyColliding = false;

        // Indicates whether the robot has entered a new collision since the previous logic update
        private bool _newCollisionSinceLastUpdate = false;

        // When the robot enters a collision (such as with a wall) Unity will only notify of the
        // collision upon initial impact. If the robot continues to drive into the wall,
        // no further collision notifications will be received. To counteract this problem, the controller will 
        // reissue the collision notification if the collision flag is not cleared and the robot is following an
        // instruction to move forward. Because the acceleration of the robot is relatively slow, the collision
        // exit may not be triggered until after a few physics updates. This variable determines how many physics
        // updates to wait before re-declaring the collision.
        private readonly int _movementUpdatesBeforeRedeclaringCollision = 2;
        private int _physicsUpdatesSinceStartingMovement = 0;
        public RobotConstraints Constraints;

        public List<(Vector3, float)> DebugCircle = new();

        public Robot2DController(Rigidbody2D rigidbody, Transform transform, Transform leftWheel, Transform rightWheel,
            MonaRobot robot)
        {
            _rigidbody = rigidbody;
            Transform = transform;
            LeftWheel = leftWheel;
            RightWheel = rightWheel;
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

        public bool HasCollidedSinceLastLogicTick()
        {
            return _newCollisionSinceLastUpdate;
        }

        public bool IsCurrentlyColliding()
        {
            return _isCurrentlyColliding;
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

        public void UpdateMotorPhysics()
        {
            // Calculate movement delta between current and last physics tick
            var leftWheelVelocityVector = LeftWheel.transform.position - _previousLeftWheelPosition ?? Vector3.zero;
            var rightWheelVelocityVector = RightWheel.transform.position - _previousRightWheelPosition ?? Vector3.zero;

            // For each wheel, determine whether it has moved forwards or backwards
            var forward = Transform.forward;
            var leftWheelMoveDirection = Vector3.Dot(forward, leftWheelVelocityVector) < 0 ? -1f : 1f;
            var rightWheelMoveDirection = Vector3.Dot(forward, rightWheelVelocityVector) < 0 ? -1f : 1f;

            // Animate rotating wheels to match movement of the robot
            AnimateWheelRotation(LeftWheel, leftWheelMoveDirection, leftWheelVelocityVector.magnitude);
            AnimateWheelRotation(RightWheel, rightWheelMoveDirection, rightWheelVelocityVector.magnitude);

            _previousLeftWheelPosition = LeftWheel.position;
            _previousRightWheelPosition = RightWheel.position;

            // Update the current status to indicate whether the robot is currently moving, stopping or idle
            if (CurrentTask != null)
            {
                // The robot is currently following an assigned task
                _currentStatus = RobotStatus.Moving;
            }
            else if (rightWheelVelocityVector.magnitude > 0.01f || leftWheelVelocityVector.magnitude > 0.01f)
            {
                // The robot is moving but is not following a task, it assumed to be in the process of stopping
                _currentStatus = RobotStatus.Stopping;
            }
            else
            {
                _currentStatus = RobotStatus.Idle;
            }

            var isAttemptingToMoveForwards = CurrentTask is MovementTask;
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
            var directive = CurrentTask?.GetNextDirective();

            if (directive != null)
                ApplyWheelForce(directive);

            // Delete task once completed
            var isCurrentTaskCompleted = CurrentTask?.IsCompleted() ?? false;
            if (isCurrentTaskCompleted)
            {
                CurrentTask = null;
            }

            if (directive != null)
                ApplyWheelForce(directive);
        }

        // The robot is rotated relative to Unity's coordinate system, so 'up' is actually forward for the robot
        public Vector3 GetForwardDirectionVector()
        {
            return Transform.up;
        }

        // Applies force at the positions of the wheels to create movement using physics simulation
        private void ApplyWheelForce(MovementDirective directive)
        {
            var leftPosition = LeftWheel.position;
            var rightPosition = RightWheel.position;

            var forward = GetForwardDirectionVector();

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
            if (_currentStatus == RobotStatus.Idle && CurrentTask != null) return RobotStatus.Moving;
            return _currentStatus;
        }

        public void Rotate(float degrees)
        {
            if (CurrentTask != null)
            {
                StopCurrentTask();
                return;
            }

            AssertRobotIsInIdleState("rotation");

            CurrentTask = new FiniteRotationTask(Transform, degrees);
        }

        public void StartRotating(bool counterClockwise = false)
        {
            if (CurrentTask != null)
            {
                StopCurrentTask();
                return;
            }

            AssertRobotIsInIdleState("rotation");
            CurrentTask = new InfiniteRotationTasK(Constraints.RelativeMoveSpeed * (counterClockwise ? -1 : 1));
        }

        public void StartRotatingAroundPoint(Vector2Int point, bool counterClockwise = false)
        {
            AssertRobotIsInIdleState("Rotating around point");
            var coarseLocation = SlamMap.CoarseMap.GetCurrentPosition();
            var angle = Vector2.SignedAngle(Geometry.DirectionAsVector(GetGlobalAngle()), Vector2.Perpendicular(point - coarseLocation) - coarseLocation);
            var radius = Vector2.Distance(Vector2Int.FloorToInt(coarseLocation), point);
            var worldPoint = SlamMap.CoarseMap.TileToWorld(point);
            var distanceBetweenWheels = Vector2.Distance(LeftWheel.position, RightWheel.position);
            DebugCircle.Add((worldPoint, radius - distanceBetweenWheels / 2));
            DebugCircle.Add((worldPoint, radius + distanceBetweenWheels / 2));
            CurrentTask = new RotateAroundPointTask(point, radius, Constraints.RelativeMoveSpeed, counterClockwise);
        }


        public void StartMoving(bool reverse = false)
        {
            AssertRobotIsInIdleState("Moving Forwards");
            CurrentTask = new MovementTask(Constraints.RelativeMoveSpeed * (reverse ? -1 : 1));
        }

        // Asserts that the current status is idle, and throws an exception if not
        protected void AssertRobotIsInIdleState(String attemptedActionName)
        {
            var currentStatus = GetStatus();
            if (currentStatus != RobotStatus.Idle)
                throw new InvalidOperationException("Tried to start action: '" + attemptedActionName
                                                                               + "' rotation action but current status is: "
                                                                               + Enum.GetName(typeof(RobotStatus),
                                                                                   currentStatus)
                                                                               + "Can only start '" +
                                                                               attemptedActionName
                                                                               + "' action when current status is Idle");
        }


        public void StopCurrentTask()
        {
            CurrentTask = null;
        }

        public void Broadcast(object data)
        {
            CommunicationManager.BroadcastMessage(_robot, data);
        }

        public List<object> ReceiveBroadcast()
        {
            return CommunicationManager.ReadMessages(_robot);
        }

        public IRobotController.DetectedWall? DetectWall(float globalAngle)
        {
            if (globalAngle < 0 || globalAngle > 360)
                throw new ArgumentException("Global angle argument must be between 0 and 360." +
                                            $"Given angle was {globalAngle}");

            var result = CommunicationManager.DetectWall(_robot, globalAngle);
            if (result != null)
            {
                var intersection = result!.Value.Item1;
                var distance = Vector2.Distance(intersection, _robot.transform.position);
                var intersectingWallAngle = result!.Value.Item2;

                // Calculate angle of wall relative to current forward angle of the robot
                var relativeWallAngle = Math.Abs(intersectingWallAngle - GetForwardAngleRelativeToXAxis());

                // Convert to relative wall angle to range 0-90
                relativeWallAngle %= 180;
                if (relativeWallAngle > 90) relativeWallAngle = 180 - relativeWallAngle;
                return new IRobotController.DetectedWall(distance, relativeWallAngle);
            }
            return null;
        }

        public string GetDebugInfo()
        {
            var info = new StringBuilder();
            var approxPosition = SlamMap.ApproximatePosition;
            info.Append($"id: {this._robot.id}\n");
            info.AppendLine($"Current task: {CurrentTask?.GetType()}");
            info.AppendLine(
                $"World Position: {Transform.position.x.ToString("#.0")}, {Transform.position.y.ToString("#.0")}");
            info.Append($"Slam tile: {SlamMap.GetCurrentPosition()}\n");
            info.Append($"Coarse tile: {SlamMap.CoarseMap.GetApproximatePosition()}\n");
            info.Append($"Is colliding: {IsCurrentlyColliding()}");
            return info.ToString();
        }

        public void Move(float distanceInMeters, bool reverse = false)
        {
            AssertRobotIsInIdleState($"Move forwards {distanceInMeters} meters");
            CurrentTask = new FiniteMovementTask(Transform, distanceInMeters, Constraints.RelativeMoveSpeed, reverse);
        }

        /// <summary>
        /// Paths and moves to the tile along the path
        /// Uses and moves along coarse tiles, handling the path by itself
        /// Must be called continuously untill the final target is reached
        /// If there is already a path, does not recompute
        /// </summary>
        /// <param name="tile">COARSEGRAINED tile as final target</param>
        public void PathAndMoveTo(Vector2Int tile)
        {
            if (GetStatus() != RobotStatus.Idle) return;
            if (_currentPath.Any() && _currentPath.Last() != tile) _currentPath.Clear();
            if (!_currentPath.Any())
            {
                var robotCurrentPosition = Vector2Int.FloorToInt(SlamMap.CoarseMap.GetApproximatePosition());
                if (robotCurrentPosition == tile) return;
                var pathList = SlamMap.CoarseMap.GetPath(tile, false, false);
                if (pathList == null) return;
                _currentPath = new Queue<Vector2Int>(pathList);
                _currentTarget = _currentPath.Dequeue();
            }
            if (SlamMap.CoarseMap.GetTileStatus(_currentTarget) == SlamMap.SlamTileStatus.Solid) _currentTarget = _currentPath.Dequeue();
            var relativePosition = SlamMap.CoarseMap.GetTileCenterRelativePosition(_currentTarget);
            if (relativePosition.Distance < 0.5f)
            {
                if (!_currentPath.Any()) return;
                _currentTarget = _currentPath.Dequeue();
                relativePosition = SlamMap.CoarseMap.GetTileCenterRelativePosition(_currentTarget);
            }
            #region DrawPath
            Debug.DrawLine(SlamMap.CoarseMap.TileToWorld(Vector2Int.FloorToInt(SlamMap.CoarseMap.GetApproximatePosition())), SlamMap.CoarseMap.TileToWorld(_currentTarget), Color.cyan, 2);
            for (int i = 0; i < _currentPath.Count-1; i++)
            {
                var pathSteps = _currentPath.ToList();
                if (i == 0)
                    Debug.DrawLine(SlamMap.CoarseMap.TileToWorld(_currentTarget), SlamMap.CoarseMap.TileToWorld(pathSteps[i]), Color.cyan, 2);
                Debug.DrawLine(SlamMap.CoarseMap.TileToWorld(pathSteps[i]), SlamMap.CoarseMap.TileToWorld(pathSteps[i + 1]), Color.cyan, 2);
            }
            if (_currentPath.Any())
            {
                var lastStep = _currentPath.Reverse().Take(2);
                Debug.DrawLine(SlamMap.CoarseMap.TileToWorld(lastStep.Last()), SlamMap.CoarseMap.TileToWorld(lastStep.First()), Color.cyan, 2);
            }
            #endregion
            if (Math.Abs(relativePosition.RelativeAngle) > 1.5f) Rotate(relativePosition.RelativeAngle);
            else if (relativePosition.Distance > 0.5f) Move(relativePosition.Distance);
        }

        /// <summary>
        /// Rotates and moves directly to target unless already moving or already on target
        /// </summary>
        /// <param name="target">COARSEGRAINED tile to move to</param>
        public void MoveTo(Vector2Int target)
        {
            var relativePosition = SlamMap.CoarseMap.GetTileCenterRelativePosition(target);
            if (GetStatus() != RobotStatus.Idle || relativePosition.Distance < 0.5f) return;
            if (Math.Abs(relativePosition.RelativeAngle) > 0.5f) Rotate(relativePosition.RelativeAngle);
            else Move(relativePosition.Distance);
        }


        public float GetGlobalAngle()
        {
            return GetForwardAngleRelativeToXAxis();
        }

        // Deposits an environment tag at the current position of the robot
        public void DepositTag(String content)
        {
            CommunicationManager.DepositTag(_robot, content);
        }

        // Returns a list of all environment tags that are within sensor range
        public List<RelativeObject<EnvironmentTag>> ReadNearbyTags()
        {
            var tags = CommunicationManager.ReadNearbyTags(_robot);
            return tags.Select(tag => ToRelativePosition(tag.MapPosition, tag)).ToList();
        }

        private RelativeObject<T> ToRelativePosition<T>(Vector2 tagPosition, T item)
        {
            var robotPosition = (Vector2)_robot.transform.position;
            var distance = Vector2.Distance(robotPosition, tagPosition);
            var angle = Vector2.SignedAngle(GetRobotDirectionVector(), tagPosition - robotPosition);
            return new RelativeObject<T>(distance, angle, item);
        }

        public List<SensedObject<int>> SenseNearbyRobots()
        {
            return CommunicationManager.SenseNearbyRobots(_robot.id)
                .Select(e => new SensedObject<int>(
                    e.Distance,
                    Vector2.SignedAngle(this._robot.transform.up,
                                                new Vector2(Mathf.Cos(e.Angle * Mathf.Deg2Rad),
                                                            Mathf.Sin(e.Angle * Mathf.Deg2Rad))),
                    e.item))
                .ToList();
        }

        public SlamMap GetSlamMap()
        {
            return this.SlamMap;
        }

        public bool IsRotating()
        {
            return CurrentTask is FiniteRotationTask || CurrentTask is InfiniteRotationTasK;
        }

        public bool IsPerformingDifferentialDriveTask()
        {
            return CurrentTask is InfiniteDifferentialMovementTask;
        }

        public bool IsRotatingIndefinitely()
        {
            return CurrentTask is InfiniteRotationTasK;
        }

        // This method requires the robot to currently be idle or already be performing an infinite rotation 
        public void RotateAtRate(float forceMultiplier)
        {
            if (forceMultiplier < -1.0f || forceMultiplier > 1.0f)
            {
                throw new ArgumentException($"Force multiplier must be in range [-1.0, 1.0]. " +
                                            $"Given value: {forceMultiplier}");
            }

            if (CurrentTask is InfiniteRotationTasK currentRotationTask)
            {
                // Adjust existing rotation task
                currentRotationTask.ForceMultiplier = Constraints.RelativeMoveSpeed * forceMultiplier;
            }
            else
            {
                // Create new rotation task
                AssertRobotIsInIdleState("infinite rotation");
                CurrentTask = new InfiniteRotationTasK(Constraints.RelativeMoveSpeed * forceMultiplier);
            }
        }

        // This method requires the robot to either be idle or already be performing an infinite movement
        public void MoveAtRate(float forceMultiplier)
        {
            if (forceMultiplier < -1.0f || forceMultiplier > 1.0f)
            {
                throw new ArgumentException($"Force multiplier must be in range [-1.0, 1.0]. " +
                                            $"Given value: {forceMultiplier}");
            }

            if (CurrentTask is MovementTask currentMovementTask)
            {
                // Adjust existing movement task
                currentMovementTask.ForceMultiplier = Constraints.RelativeMoveSpeed * forceMultiplier;
            }
            else
            {
                // Create new movement task
                AssertRobotIsInIdleState("Infinite movement");
                CurrentTask = new MovementTask(Constraints.RelativeMoveSpeed * forceMultiplier);
            }
        }

        // This method allows for differential drive (each wheel is controlled separately)
        public void SetWheelForceFactors(float leftWheelForce, float rightWheelForce)
        {
            // Apply force multiplier from robot constraints (this value varies based on robot size)
            leftWheelForce *= Constraints.RelativeMoveSpeed;
            rightWheelForce *= Constraints.RelativeMoveSpeed;

            if (CurrentTask is InfiniteDifferentialMovementTask existingTask)
            {
                // Update the existing differential movement task
                existingTask.UpdateWheelForces(leftWheelForce, rightWheelForce);
            }
            else
            {
                // The robot must be in idle state to start this task
                AssertRobotIsInIdleState("Differential movement");
                CurrentTask = new InfiniteDifferentialMovementTask(leftWheelForce, rightWheelForce);
            }
        }
    }
}