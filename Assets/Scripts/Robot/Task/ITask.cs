namespace Maes.Robot.Task {
    public interface ITask {
        MovementDirective GetNextDirective();
        bool IsCompleted();
    }
}