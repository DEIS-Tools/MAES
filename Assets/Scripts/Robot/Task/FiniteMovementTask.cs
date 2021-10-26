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
            if (_isCompleted)
                return MovementDirective.NoMovement;
            
            float remainingDistance =  _targetDistance - Vector2.Distance(_startingPosition, _robotTransform.position);
            if (remainingDistance > 0.1f)
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
            int stopTimeTicks = GetStopTime(currentVelocity);
            float stopDistance = GetDistanceTraveled(currentVelocity, stopTimeTicks);
            if (stopDistance <= remainingDistance - 0.01f) return 1.0f;
            else return 0f;
        }

        // Returns the time (in ticks from now) at which the velocity of the robot will be approximately 0 (<0.001) 
        private int GetStopTime(float currentVelocity)
        {
            return (int) (11f * (Mathf.Log(currentVelocity) + 3 * Mathf.Log(10)) / 2f);
        }

        // Returns the distance traveled in the given ticks when starting at the given velocity
        private float GetDistanceTraveled(float currentVelocity, int ticks)
        {
            // Get offset by solving for C in:
            // 0 = (-11/2)*v0*e^(-t*2/11)+C
            var offset = (float) ((11f * currentVelocity * Math.Pow(Math.E, (-2 / 11) * 0)) / 2f); 
            return (float) ((11f * currentVelocity * Math.Pow(Math.E, (-2 / 11) * ticks)) / 2f) + offset; 
        }

        public bool IsCompleted()
        {
            //Debug.Log($"Distance traveled: {Vector2.Distance(_startingPosition, _robotTransform.position)}");
            return _isCompleted; 
        }
    }
}