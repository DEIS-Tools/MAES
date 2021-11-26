namespace Maes.Robot.Task {
    public class InfiniteRotationTasK : ITask {
        private readonly bool _counterClockWise;

        public InfiniteRotationTasK(bool counterClockWise) {
            _counterClockWise = counterClockWise;
        }

        public MovementDirective GetNextDirective() {
            return _counterClockWise ? MovementDirective.Left : MovementDirective.Right;
        }

        public bool IsCompleted() {
            return false;
        }
    }
}