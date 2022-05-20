using Maes.ExplorationAlgorithm;
using Maes.Robot;

namespace PlayModeTests {
    public class TestingAlgorithm : IExplorationAlgorithm {
        public int Tick = 0;
        public Robot2DController Controller;
        public CustomUpdateFunction UpdateFunction = (tick, controller) => { };

        public delegate void CustomUpdateFunction(int tick, Robot2DController controller);

        private CustomUpdateFunction onUpdate;
        public TestingAlgorithm() {
            this.onUpdate = (_, __) => { };
        }
        public object SaveState() { throw new System.NotImplementedException(); }
        public void RestoreState(object stateInfo) { }

        public void UpdateLogic() {
            UpdateFunction(Tick, Controller);
            Tick++;
        }

        public void SetController(Robot2DController controller) {
            this.Controller = controller;
        }

        public string GetDebugInfo() {
            return "";
        }
    }
}