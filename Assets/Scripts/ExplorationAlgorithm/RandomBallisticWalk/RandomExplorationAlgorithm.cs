using Maes.Robot;
using Maes.Robot.Task;
using Random = System.Random;

namespace Maes.ExplorationAlgorithm.RandomBallisticWalk {
    public class RandomExplorationAlgorithm : IExplorationAlgorithm {
        private IRobotController _robotController;
        private bool _hasJustRotated = true;
        private readonly Random _random;

        public RandomExplorationAlgorithm(int randomSeed) {
            _random = new Random(randomSeed);
        }

        public RandomExplorationAlgorithm(Robot2DController robotControllerController, int randomSeed) {
            _robotController = robotControllerController;
            _random = new Random(randomSeed);
        }

        public void UpdateLogic() {
            //_robotController.ReadNearbyTags();
            var status = _robotController.GetStatus();
            if (status == RobotStatus.Idle) {
                if (!_hasJustRotated) {
                    var direction = _random.Next(0, 1) == 0 ? -1 : 1;
                    var degrees = _random.Next(50, 180);
                    _robotController.Rotate(degrees * direction);
                    _hasJustRotated = true;
                }
                else {
                    _robotController.StartMoving();
                    _hasJustRotated = false;
                }
            }
        }

        public void SetController(Robot2DController controller) {
            this._robotController = controller;
        }

        public string GetDebugInfo() {
            return "";
        }

        public object SaveState() {
            throw new System.NotImplementedException();
        }

        public void RestoreState(object stateInfo) {
            throw new System.NotImplementedException();
        }
    }
}