namespace Maes.Robot.Task {
    public class InfiniteRotationTasK : ITask {
        private readonly bool _counterClockWise;
        private readonly float _force;

        public InfiniteRotationTasK(bool counterClockWise, float force) {
            _counterClockWise = counterClockWise;
            _force = force;
        }

        public MovementDirective GetNextDirective() {
            return _counterClockWise ? MovementDirective.Left(_force) : MovementDirective.Right(_force);
        }

        public bool IsCompleted() {
            return false;
        }
    }
}