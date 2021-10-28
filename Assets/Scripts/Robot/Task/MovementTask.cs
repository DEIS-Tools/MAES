namespace Dora.Robot.Task {
    public class MovementTask : ITask {
        private readonly bool reverse;

        public MovementTask(bool reverse = false) {
            this.reverse = reverse;
        }

        public MovementDirective GetNextDirective() {
            if (reverse) return MovementDirective.Reverse;
            else return MovementDirective.Forward;
        }

        public bool IsCompleted() {
            return false;
        }
    }
}