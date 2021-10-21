namespace Dora.ExplorationAlgorithm
{
    public interface IExplorationAlgorithm: ISavable<object>
    {

        public void UpdateLogic();

        public void SetController(Robot2DController controller);
    }
}