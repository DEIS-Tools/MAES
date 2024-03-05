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
using UnityEngine;

namespace Maes.Robot.Task {
    internal class FiniteMovementTask : ITask {
        private readonly float _targetDistance;
        private readonly Transform _robotTransform;
        private readonly bool _reverse;

        private readonly Vector2 _startingPosition;
        private Vector2 _previousPosition;
        private bool _isCompleted = false;
        private readonly float _force;

        public FiniteMovementTask(Transform robotTransform, float targetDistance, float force, bool reverse = false) {
            _reverse = reverse;
            _targetDistance = targetDistance;
            _robotTransform = robotTransform;
            _startingPosition = robotTransform.position;
            _previousPosition = _startingPosition;
            _force = force;
        }

        public MovementDirective GetNextDirective() {
            if (_isCompleted)
                return MovementDirective.NoMovement();

            float remainingDistance = _targetDistance - Vector2.Distance(_startingPosition, _robotTransform.position);
            if (remainingDistance > 0.1f) {
                var currentPosition = _robotTransform.position;
                var currentVelocity = Vector2.Distance(_previousPosition, currentPosition);
                _previousPosition = currentPosition;
                var forceFactor = GetForceFactor(remainingDistance, currentVelocity);
                if (_reverse) forceFactor *= -1f;
                return new MovementDirective(forceFactor, forceFactor);
            }
            else {
                _isCompleted = true;
                return MovementDirective.NoMovement();
            }
        }

        // The applied force depends on how large a distance is remaining and how fast the robot is currently moving
        private float GetForceFactor(float remainingDistance, float currentVelocity) {
            int stopTimeTicks = GetStopTime(currentVelocity);
            float stopDistance = GetDistanceTraveled(currentVelocity, stopTimeTicks);
            if (stopDistance <= remainingDistance - 0.01f) return _force;
            else return 0f;
        }

        // Returns the time (in ticks from now) at which the velocity of the robot will be approximately 0 (<0.001) 
        private int GetStopTime(float currentVelocity) {
            return (int) (11f * (Mathf.Log(currentVelocity) + 3 * Mathf.Log(10)) / 2f);
        }

        // Returns the distance traveled in the given ticks when starting at the given velocity
        private float GetDistanceTraveled(float currentVelocity, int ticks) {
            // Get offset by solving for C in:
            // 0 = (-11/2)*v0*e^(-t*2/11)+C
            var offset = (float) ((11f * currentVelocity * Math.Pow(Math.E, (-2 / 11) * 0)) / 2f);
            return (float) ((11f * currentVelocity * Math.Pow(Math.E, (-2 / 11) * ticks)) / 2f) + offset;
        }

        public bool IsCompleted() {
            //Debug.Log($"Distance traveled: {Vector2.Distance(_startingPosition, _robotTransform.position)}");
            return _isCompleted;
        }
    }
}