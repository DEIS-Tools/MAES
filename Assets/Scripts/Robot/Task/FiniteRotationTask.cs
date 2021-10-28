using System;
using UnityEngine;

namespace Dora.Robot.Task {
    // Represents a task to rotate the robot by a given amount of degrees
    public class FiniteRotationTask : ITask {
        private readonly float _degreesToRotate;
        private readonly float _startingAngle;

        private readonly Transform _robotTransform;
        private float _previousRotation = 0f;

        private bool _isCompleted = false;
        public bool IsCompleted() => _isCompleted;

        public FiniteRotationTask(Transform robotTransform, float degreesToRotate) {
            if (degreesToRotate < -180f || degreesToRotate > 180f)
                throw new ArgumentException($"Given rotation must be between -180° and 180° " +
                                            $"but target rotation was {degreesToRotate}");
            _degreesToRotate = degreesToRotate;
            _robotTransform = robotTransform;
            _startingAngle = robotTransform.rotation.eulerAngles.z;
        }

        public MovementDirective GetNextDirective() {
            if (_isCompleted) return MovementDirective.NoMovement;

            // Find the current amount of rotation since starting the task
            var absRotation = GetAbsoluteDegreesRotated();

            // Find the speed of the rotation during the previous tick
            var currentRotationRate = absRotation - _previousRotation;
            _previousRotation = absRotation;

            // Calculate how much more we need to rotate before reaching the goal
            var remainingRotation = Math.Abs(_degreesToRotate) - absRotation;

            // Calculate how often we need 
            int stopTimeTicks = GetStopTime(currentRotationRate);
            float degreesRotatedBeforeStop = GetDegreesRotated(currentRotationRate, stopTimeTicks);

            // Calculate how far the robot is from reaching the target rotation if we stop applying force now
            var targetDelta = remainingRotation - degreesRotatedBeforeStop;

            var forceMultiplier = 1f;
            if (targetDelta < 2.28f) {
                // We need to apply some amount of force to rotate the last amount
                // It has been observed that if we apply maximum force (1.0) we will rotate an additional 2.28 degrees 
                // To find the appropriate amount of force, use linear interpolation on the target delta. 
                forceMultiplier = (1f / 2.28f) * targetDelta * 0.85f; // 0.85 is a magic number, sorry
            }

            if (targetDelta < 0.1f) {
                // The robot will be within acceptable range when rotation has stopped by itself.
                // Stop applying force and consider task completed
                forceMultiplier = 0;
                _isCompleted = true;
            }

            // Determine rotation direction
            if (_degreesToRotate < 0) forceMultiplier *= -1;

            return new MovementDirective(forceMultiplier, -forceMultiplier);
        }


        // Returns the time (in ticks from now) at which the velocity of the robot will be approximately 0 (<0.001) 
        private int GetStopTime(float currentRotationRate) {
            return (int) (-3.81f * Mathf.Log(0.01f / currentRotationRate));
        }

        // Returns the degrees rotated over the given ticks when starting at the given rotation rate
        private float GetDegreesRotated(float currentRotationRate, int ticks) {
            // Get offset by solving for C in:
            // 0 = -3.81*v0*e^(-t/3.81)+C
            //var offset = (float) (3.81 * currentRotationRate * Math.Pow(Math.E, -(1f / 3.81f) * 0));
            //return (float) (-3.81 * currentRotationRate * Math.Pow(Math.E, -(1f / 3.81f) * ticks) + offset) - currentRotationRate;

            var rotation = 0f;
            for (int i = 0; i < ticks; i++) {
                rotation += GetRotationRate(currentRotationRate, i + 1);
            }

            return rotation;
        }

        // Returns the speed of rotaion (measured in degrees per tick) after waiting the given amount of ticks without
        // applying force
        private float GetRotationRate(float startingRate, int ticks) {
            return (float) (startingRate * Math.Pow(Math.E, -ticks / 3.81f));
        }


        // Returns the amount of degrees that has been rotated since starting this task
        private float GetAbsoluteDegreesRotated() {
            return Math.Abs(Mathf.DeltaAngle(_robotTransform.rotation.eulerAngles.z, _startingAngle));
        }
    }
}