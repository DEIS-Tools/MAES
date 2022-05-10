using System;

namespace Maes.Robot.Task {
    public class InfiniteRotationTasK : ITask {
        public float ForceMultiplier;

        public InfiniteRotationTasK(float forceMultiplier) {
            ForceMultiplier = forceMultiplier;
        }

        public MovementDirective GetNextDirective() {
            var absMultiplier = Math.Abs(ForceMultiplier);
            return ForceMultiplier < 0 ? MovementDirective.Left(absMultiplier) : MovementDirective.Right(absMultiplier);
        }

        public bool IsCompleted() {
            return false;
        }
    }
}