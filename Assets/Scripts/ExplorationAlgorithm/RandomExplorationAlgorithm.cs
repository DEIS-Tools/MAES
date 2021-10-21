using System;
using Dora.Robot;

namespace Dora.ExplorationAlgorithm
{
    public class RandomExplorationAlgorithm: IExplorationAlgorithm
    {
        
        private Robot2DController _robotController;
        private bool _hasJustRotated = false;
        private Random _random;

        
        public RandomExplorationAlgorithm(int randomSeed)
        {
            _random = new Random(randomSeed);
        }
        public RandomExplorationAlgorithm(Robot2DController robotControllerController, int randomSeed)
        {
            _robotController = robotControllerController;
            _random = new Random(randomSeed);
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
            // Testing
            _robotController.ReceiveBroadcast();
            
            // Testing
            _robotController.Broadcast("Test!");
            
            var status = _robotController.GetStatus();
            if (status == RobotStatus.Idle)
            {
                if (!_hasJustRotated)
                {
                    var direction = _random.Next(0, 1) == 0 ? -1 : 1;
                    var degrees = _random.Next(50, 180);
                    _robotController.Rotate(degrees * direction);
                    _hasJustRotated = true;
                    
                }
                else
                {
                    _robotController.StartMovingForwards();
                    _hasJustRotated = false;
                }
            }
        }

        public void SetController(Robot2DController controller) {
            this._robotController = controller;
        }
    }
}