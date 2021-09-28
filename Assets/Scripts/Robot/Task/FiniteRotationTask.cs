using System;
using UnityEngine;

namespace Dora.Robot.Task
{
    // Represents a task to rotate the robot by a given amount of degrees
    public class FiniteRotationTask: ITask
    {

        private readonly float _degreesToRotate;
        private readonly Transform _transform;
        
        private readonly float _startingAngle;
        private bool _isCompleted = false;
        // The robot stops applying force when the close enough to target rotation
        // The point at which force application should stop depends on how far the robot should be rotated,
        // as the rotation might never reach max velocity if the rotation distance is short enough
        private readonly float _forceApplicationStopAngle;
        
        public FiniteRotationTask(Transform transform, float degreesToRotate)
        {
            _degreesToRotate = degreesToRotate;
            
            var absRotation = Math.Abs(degreesToRotate);
            
            // !Disclaimer!: This is not the prettiest solution, but it works for now.
            // In the future this should be replaced by a force calculation that determines 
            // how much force to apply based on remaining rotation and current angular velocity
            
            // By default stop applying force when within 5.0 degrees of the target.
            // This results in stopping **roughly** at the target
            _forceApplicationStopAngle = 5.0f;
            // Special cases for lower rotation amounts, where maximum angular velocity is never reached
            if (absRotation <= 2.0f) 
                _forceApplicationStopAngle = Math.Max(0.2f, absRotation - 0.4f);
            else if (absRotation <= 3.0f)
                _forceApplicationStopAngle = 2.5f;
            else if (absRotation <= 5.5f)
                _forceApplicationStopAngle = 3.0f;
            else if (absRotation <= 7.0f) 
                _forceApplicationStopAngle = 4.0f;
            else if (absRotation <= 9.2f) 
                _forceApplicationStopAngle = 4.5f; 

            _transform = transform;
            _startingAngle = transform.rotation.eulerAngles.z;
        }

        public MovementDirective GetNextDirective()
        {
            if (_isCompleted) return MovementDirective.NoMovement;

            var absDegreesRotated = GetAbsoluteDegreesRotated();
            // If near target rotation, stop rotation by applying force in the opposite direction
            if (Math.Abs(_degreesToRotate) - absDegreesRotated <= _forceApplicationStopAngle)
            {
                _isCompleted = true;
               /* if (_degreesToRotate < 0) return MovementDirective.Right;
                else return MovementDirective.Left;*/
               return MovementDirective.NoMovement;

            }
            // Otherwise keep rotating
            if (_degreesToRotate < 0) return MovementDirective.Left;
            else return MovementDirective.Right;
        }
        
        
        public bool IsCompleted()
        {
            return _isCompleted;
        }

        // Returns the amount of degrees that has been rotated since starting this task
        private float GetAbsoluteDegreesRotated()
        {
            return Math.Abs(Mathf.DeltaAngle(_transform.rotation.eulerAngles.z, _startingAngle));
        }

    }
}