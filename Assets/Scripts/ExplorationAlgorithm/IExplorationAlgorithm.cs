namespace Dora.ExplorationAlgorithm
{
    public interface IExplorationAlgorithm: ISavable<object>
    {

        public void UpdateLogic(SimulationConfiguration configuration);

        public void SetController(Robot2DController controller);
    }
}