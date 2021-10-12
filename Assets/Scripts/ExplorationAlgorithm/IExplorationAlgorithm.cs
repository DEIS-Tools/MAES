namespace Dora.ExplorationAlgorithm
{
    public interface IExplorationAlgorithm: ISavable<object>
    {

        public void UpdateLogic(SimulationConfiguration configuration);

    }
}