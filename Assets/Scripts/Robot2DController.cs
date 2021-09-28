using System.Collections;
using System.Collections.Generic;
using Dora.Robot;
using UnityEngine;

namespace Dora
{
    public class Robot2DController : MonoBehaviour, ISimulationUnit
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

            if (directive != null)
                ApplyWheelForce(directive);
        }
        
        // Applies force at the positions of the wheels to create movement using physics simulation
        private void ApplyWheelForce(MovementDirective directive)
        {
            var leftPosition = leftWheel.position;
            var rightPosition = rightWheel.position;

            var forward = transform.forward;
            Debug.Log("Forward: " + (Vector2) forward);
            
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
    }
}
