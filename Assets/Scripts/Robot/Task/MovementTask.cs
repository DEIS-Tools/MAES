using UnityEngine;

namespace Maes.Robot.Task {
    public class MovementTask : ITask {
        public float ForceMultiplier;

        public MovementTask(float forceMultiplier) {
            ForceMultiplier = forceMultiplier;
        }

        public MovementDirective GetNextDirective() {
            var absForce = Mathf.Abs(ForceMultiplier);
            if (ForceMultiplier < 0) return MovementDirective.Reverse(absForce);
            else return MovementDirective.Forward(absForce);
        }

        public bool IsCompleted() {
            return false;
        }
    }
}