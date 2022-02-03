using System;
using UnityEngine;

namespace Maes.Robot {
    public class MovementDirective {
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