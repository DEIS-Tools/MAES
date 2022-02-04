namespace Maes.Robot.Task {
    public class MovementTask : ITask {
        private readonly bool _reverse;
        private readonly float _force;

        public MovementTask(float force, bool reverse = false) {
            _reverse = reverse;
            _force = force;
        }

        public MovementDirective GetNextDirective() {
            if (_reverse) return MovementDirective.Reverse(_force);
            else return MovementDirective.Forward(_force);
        }

        public bool IsCompleted() {
            return false;
        }
    }
}