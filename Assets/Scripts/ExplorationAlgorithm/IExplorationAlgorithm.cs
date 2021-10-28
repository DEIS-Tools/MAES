namespace Dora.ExplorationAlgorithm {
    public interface IExplorationAlgorithm : ISavable<object> {
        public void UpdateLogic();

        public void SetController(Robot2DController controller);

        // Returns debug info that will be shown when the robot is selected
        public string GetDebugInfo();
    }
}