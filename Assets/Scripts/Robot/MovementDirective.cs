using System;
using UnityEngine;

namespace Maes.Robot {
    public class MovementDirective {
        public readonly float LeftWheelSpeed;
        public readonly float RightWheelSpeed;

        public static readonly MovementDirective Left = new MovementDirective(-1f, 1f);
        public static readonly MovementDirective Right = new MovementDirective(1f, -1f);
        public static readonly MovementDirective Forward = new MovementDirective(1f, 1f);
        public static readonly MovementDirective Reverse = new MovementDirective(-1f, -1f);
        public static readonly MovementDirective NoMovement = new MovementDirective(0f, 0f);

        public MovementDirective(float leftWheelSpeed, float rightWheelSpeed) {
            if (leftWheelSpeed > 1.0f || leftWheelSpeed < -1.0f || rightWheelSpeed > 1.0f || rightWheelSpeed < -1.0f)
                throw new ArgumentException($"Left and right wheel speeds must be within bounds [-1.0, 1.0]" +
                                            $" but given left/right speeds were {leftWheelSpeed} and {rightWheelSpeed}");

            RightWheelSpeed = rightWheelSpeed;
            LeftWheelSpeed = leftWheelSpeed;
        }

        public bool IsRotational() {
            return Mathf.Abs(LeftWheelSpeed - RightWheelSpeed) > 0.01f;
        }
    }
}