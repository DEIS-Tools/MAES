using System;

namespace Dora.Robot.ExplorationAlgorithm
{
    public class RandomExplorationAlgorithm: IExplorationAlgorithm
    {
        
        private Robot _robot;
        private bool _hasJustRotated = false;
        private Random _random;

        public RandomExplorationAlgorithm(Robot robot) // Todo: Random seed
        {
            _robot = robot;
            _random = new Random();
        }
        
        public object SaveState()
        {
            throw new System.NotImplementedException();
        }

        public void RestoreState(object stateInfo)
        {
            throw new System.NotImplementedException();
        }

        public void UpdateLogic(SimulationConfiguration config)
        {
            var controller = _robot.movementController;
            var status = controller.GetStatus();
            if (status == RobotStatus.Idle)
            {
                if (!_hasJustRotated)
                {
                    var direction = _random.Next(0, 1) == 0 ? -1 : 1;
                    var degrees = _random.Next(50, 180);
                    controller.Rotate(degrees * direction);
                    _hasJustRotated = true;
                }
                else
                {
                    controller.MoveForward();
                    _hasJustRotated = false;
                }
            }
        }
    }
}