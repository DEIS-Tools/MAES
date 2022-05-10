namespace Maes.Robot.Task {
    
    /// Represents a task where the force application at each wheel may be controlled individually
    /// This allows for rotation while moving ahead 
    public class InfiniteDifferentialMovementTask : ITask {

        private float _leftWheelForce = 0f;
        private float _rightWheelForce = 0f;

        public InfiniteDifferentialMovementTask(float leftWheelForce, float rightWheelForce) {
            UpdateWheelForces(leftWheelForce, rightWheelForce);
        }

        public void UpdateWheelForces(float leftWheelForce, float rightWheelForce) {
            _leftWheelForce = leftWheelForce;
            _rightWheelForce = rightWheelForce;
        }

        public MovementDirective GetNextDirective() {
            return new MovementDirective(_leftWheelForce, _rightWheelForce);
        }

        public bool IsCompleted() {
            // This is an infinite movement task that can only be terminated manually
            return false;
        }
    }
}