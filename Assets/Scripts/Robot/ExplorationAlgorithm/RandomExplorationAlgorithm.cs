using System;

namespace Dora.Robot.ExplorationAlgorithm
{
    public class RandomExplorationAlgorithm: IExplorationAlgorithm
    {
        
        private IRobotController _controller;
        private bool _hasJustRotated = false;
        private Random _random;

        public RandomExplorationAlgorithm(IRobotController controller) // Todo: Random seed
        {
            _controller = controller;
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
            var status = _controller.GetStatus();
            if (status == RobotStatus.Idle)
            {
                if (!_hasJustRotated)
                {
                    var direction = _random.Next(0, 1) == 0 ? -1 : 1;
                    var degrees = _random.Next(50, 180);
                    _controller.Rotate(degrees * direction);
                    _hasJustRotated = true;
                }
                else
                {
                    _controller.MoveForward();
                    _hasJustRotated = false;
                }
            }
        }
    }
}