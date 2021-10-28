namespace Dora.Robot.Task {
    public interface ITask {
        MovementDirective GetNextDirective();
        bool IsCompleted();
    }
}