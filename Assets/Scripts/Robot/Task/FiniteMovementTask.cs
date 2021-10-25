using System;
using UnityEngine;

namespace Dora.Robot.Task
{
    public class FiniteMovementTask: ITask
    {
        private readonly float _targetDistance;
        private readonly Transform _robotTransform;
        private readonly bool _reverse;

        private readonly Vector2 _startingPosition;
        private Vector2 _previousPosition;
        private bool _isCompleted = false;

        public FiniteMovementTask( Transform robotTransform, float targetDistance, bool reverse = false)
        {
            _reverse = reverse;
            _targetDistance = targetDistance;
            _robotTransform = robotTransform;
            _startingPosition = robotTransform.position;
            _previousPosition = _startingPosition;
        }
        
        public MovementDirective GetNextDirective()
        {
            float remainingDistance =  _targetDistance - Vector2.Distance(_startingPosition, _robotTransform.position);
            if (remainingDistance > 0 || true)
            {
                var currentPosition = _robotTransform.position;
                var currentVelocity = Vector2.Distance(_previousPosition, currentPosition);
                _previousPosition = currentPosition;
                var forceFactor = GetForceFactor(remainingDistance, currentVelocity);
                if (_reverse) forceFactor *= -1f;
                return new MovementDirective(forceFactor, forceFactor);
            }
            else
            {
                _isCompleted = true;
                return MovementDirective.NoMovement;
            }

        }
        
        // The applied force depends on how large a distance is remaining and how fast the robot is currently moving
        private float GetForceFactor(float remainingDistance, float currentVelocity)
        {
            Debug.Log($"Velocity: {currentVelocity}, distance: {remainingDistance}");
            if (remainingDistance <= 0.0f) return 0.0f;
            //if (remainingDistance < 0.05f) return 0.0f;
            //if (remainingDistance < 0.1f) return 0.5f;
            return 1.0f;
        }

        public bool IsCompleted()
        {
            Debug.Log($"Distance traveled: {Vector2.Distance(_startingPosition, _robotTransform.position)}");
            return false; //_isCompleted || Vector2.Distance(_startingPosition, _robotTransform.position) >= _targetDistance;
        }
    }
}