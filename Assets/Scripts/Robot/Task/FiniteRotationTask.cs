using System;
using UnityEngine;

namespace Dora.Robot.Task
{
    // Represents a task to rotate the robot by a given amount of degrees
    public class FiniteRotationTask: ITask
    {
        private readonly float _degreesToRotate;
        private readonly Transform _robotTransform;
        
        private readonly float _startingAngle;
        private bool _isCompleted = false;

        private float _previousRotation = 0f;
        
        // The robot stops applying force when the close enough to target rotation
        // The point at which force application should stop depends on how far the robot should be rotated,
        // as the rotation might never reach max velocity if the rotation distance is short enough
        private readonly float _forceApplicationStopAngle;
        
        public FiniteRotationTask(Transform robotTransform, float degreesToRotate)
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

            _robotTransform = robotTransform;
            _startingAngle = robotTransform.rotation.eulerAngles.z;
        }

        public MovementDirective GetNextDirective()
        {
            if (_isCompleted) return MovementDirective.NoMovement;
            
            var absRotation = GetAbsoluteDegreesRotated();
            var currentRotationRate = absRotation - _previousRotation;
            _previousRotation = absRotation;
            
            var remainingRotation = Math.Abs(_degreesToRotate) - absRotation; 
            if (currentRotationRate > 0)
            {
                int stopTimeTicks = GetStopTime(currentRotationRate);
                float degreesRotatedBeforeStop = GetDegreesRotated(currentRotationRate, stopTimeTicks);
                Debug.Log($"Current rotation: {absRotation} and rotation rate: {currentRotationRate}\nTicks before stopping: {stopTimeTicks}. Degrees rotated before stopping: {degreesRotatedBeforeStop}");
                if (degreesRotatedBeforeStop <= remainingRotation) return _degreesToRotate < 0 ? MovementDirective.Left : MovementDirective.Right;
                else
                {
                    _isCompleted = true;
                    return MovementDirective.NoMovement;
                }
            }
            else
            {
                return _degreesToRotate < 0 ? MovementDirective.Left : MovementDirective.Right;
            }
            

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
            Debug.Log($"Degrees turned: {GetAbsoluteDegreesRotated()}");
            return false;
            return _isCompleted;
        }
        
        // Returns the time (in ticks from now) at which the velocity of the robot will be approximately 0 (<0.001) 
        private int GetStopTime(float currentRotationRate)
        {
            return (int) (-3.81f * Mathf.Log(0.01f/currentRotationRate));
        }
        
        // Returns the degrees rotated over the given ticks when starting at the given rotation rate
        private float GetDegreesRotated(float currentRotationRate, int ticks)
        {
            // Get offset by solving for C in:
            // 0 = -3.81*v0*e^(-t/3.81)+C
            //var offset = (float) (3.81 * currentRotationRate * Math.Pow(Math.E, -(1f / 3.81f) * 0));
            //return (float) (-3.81 * currentRotationRate * Math.Pow(Math.E, -(1f / 3.81f) * ticks) + offset) - currentRotationRate;

            var rotation = 0f;
            for (int i = 0; i < ticks; i++)
            {
                rotation += GetRotationRate(currentRotationRate, i + 1);
            }

            return rotation;
        }

        private float GetRotationRate(float startingRate, int ticks)
        {
            return (float) (startingRate * Math.Pow(Math.E, -ticks / 3.81f));
        }
        

        // Returns the amount of degrees that has been rotated since starting this task
        private float GetAbsoluteDegreesRotated()
        {
            return Math.Abs(Mathf.DeltaAngle(_robotTransform.rotation.eulerAngles.z, _startingAngle));
        }

    }
}