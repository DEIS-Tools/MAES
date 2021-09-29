namespace Dora.Robot.ExplorationAlgorithm
{
    public interface IExplorationAlgorithm: ISavable<object>
    {

        public void UpdateLogic(SimulationConfiguration configuration);

    }
}