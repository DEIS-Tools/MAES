using System;
using UnityEngine;

namespace Dora.Robot.Task
{
    public class FiniteRotationTask: ITask
    {

        private readonly float _targetRotation;
        private readonly Transform _transform;
        
        private readonly float _startingAngle;
        private bool _isCompleted = false;
        
        public FiniteRotationTask(Transform transform, float targetRotation)
        {
            _targetRotation = targetRotation;
            _transform = transform;
            _startingAngle = transform.rotation.eulerAngles.y;
        }

        public MovementDirective GetNextDirective()
        {
            if (_isCompleted) return MovementDirective.NoMovement;

            var degreesRotated = GetAbsoluteDegreesRotated();
            // If near target rotation, stop rotation by applying force in the opposite direction
            if (Math.Abs(_targetRotation) - 8f < degreesRotated)
            {
                _isCompleted = true;
                if (_targetRotation < 0) return MovementDirective.Right;
                else return MovementDirective.Left;
            }
            // Otherwise keep rotating
            if (_targetRotation < 0) return MovementDirective.Left;
            else return MovementDirective.Right;
        }
        
        
        public bool IsCompleted()
        {
            return _isCompleted;
        }

        // Returns the amount of degrees that has been rotated since starting this task
        private float GetAbsoluteDegreesRotated()
        {
            return Math.Abs(Mathf.DeltaAngle(_transform.rotation.eulerAngles.y, _startingAngle));
        }

    }
}