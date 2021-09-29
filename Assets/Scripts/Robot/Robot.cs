using Dora.Robot.ExplorationAlgorithm;

namespace Dora.Robot
{
    public class Robot: ISimulationUnit
    {
        // The algorithm that control the logic of the robot
        private readonly IExplorationAlgorithm _algorithm;
        
        // The controller that provides an interface for moving the robot
        private readonly IRobotController _movementController;
        
        public object SaveState()
        {
            throw new System.NotImplementedException();
        }

        public void RestoreState(object stateInfo)
        {
            throw new System.NotImplementedException();
        }

        public void LogicUpdate(SimulationConfiguration config)
        {
            _algorithm.UpdateLogic(config);
        }

        public void PhysicsUpdate(SimulationConfiguration config)
        {
            _movementController.UpdateMotorPhysics(config);
        }
    }
}