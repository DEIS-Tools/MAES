using Dora.Robot;

namespace Dora.ExplorationAlgorithm.TheNextFrontier {
    public class TnfExplorationAlgorithm : IExplorationAlgorithm {
        private IRobotController _robotController;

        public void UpdateLogic() {
            if (_robotController.GetStatus() == RobotStatus.Idle) {
                _robotController.Move(5);
            }
        }

        public void SetController(Robot2DController controller) {
            _robotController = controller;
        }

        public string GetDebugInfo() {
            throw new System.NotImplementedException();
        }




        // FUTURE WORK
        public object SaveState() {
            throw new System.NotImplementedException();
        }

        public void RestoreState(object stateInfo) {
            throw new System.NotImplementedException();
        }
    }
}