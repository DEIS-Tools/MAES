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

namespace Maes.Robot {
    internal class MovementDirective {
        public readonly float LeftWheelSpeed;
        public readonly float RightWheelSpeed;
        
        public static MovementDirective Left(float force) {
            return new MovementDirective(-force, force);
        }

        public static MovementDirective Right(float force) {
            return new MovementDirective(force, -force);
        }

        public static MovementDirective Forward(float force) {
            return new MovementDirective(force, force);
        }
        
        public static MovementDirective Reverse(float force) {
            return new MovementDirective(-force, -force);
        }
        
        public static MovementDirective NoMovement() {
            return new MovementDirective(0f, 0f);
        }

        public MovementDirective(float leftWheelSpeed, float rightWheelSpeed) {
            RightWheelSpeed = rightWheelSpeed;
            LeftWheelSpeed = leftWheelSpeed;
        }

        public bool IsRotational() {
            return Mathf.Abs(LeftWheelSpeed - RightWheelSpeed) > 0.01f;
        }
    }
}