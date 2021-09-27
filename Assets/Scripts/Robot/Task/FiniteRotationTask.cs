namespace Dora.Robot.Task
{
    public class FiniteRotationTask: ITask
    {

        private readonly float _targetRotation;
        
        private int tickCount = 0;
        private int targetTickCount = 20;
        
        public FiniteRotationTask(float targetRotation)
        {
            _targetRotation = targetRotation;
        }

        public MovementDirective GetNextDirective()
        {
            tickCount++;
            if (_targetRotation < 0) return MovementDirective.Left;
            else return MovementDirective.Right;
        }

        public bool IsCompleted()
        {
            return tickCount >= targetTickCount;
        }
    }
}