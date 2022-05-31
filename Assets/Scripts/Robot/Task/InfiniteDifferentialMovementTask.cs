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

namespace Maes.Robot.Task {
    
    /// Represents a task where the force application at each wheel may be controlled individually
    /// This allows for rotation while moving ahead 
    public class InfiniteDifferentialMovementTask : ITask {

        private float _leftWheelForce = 0f;
        private float _rightWheelForce = 0f;

        public InfiniteDifferentialMovementTask(float leftWheelForce, float rightWheelForce) {
            UpdateWheelForces(leftWheelForce, rightWheelForce);
        }

        public void UpdateWheelForces(float leftWheelForce, float rightWheelForce) {
            _leftWheelForce = leftWheelForce;
            _rightWheelForce = rightWheelForce;
        }

        public MovementDirective GetNextDirective() {
            return new MovementDirective(_leftWheelForce, _rightWheelForce);
        }

        public bool IsCompleted() {
            // This is an infinite movement task that can only be terminated manually
            return false;
        }
    }
}