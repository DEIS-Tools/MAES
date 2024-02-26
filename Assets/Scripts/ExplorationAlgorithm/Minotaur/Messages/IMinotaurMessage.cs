namespace Maes.ExplorationAlgorithm.Minotaur
{
    public interface IMinotaurMessage
    {
        public IMinotaurMessage Process(MinotaurAlgorithm minotaur);
        public IMinotaurMessage Combine(IMinotaurMessage otherMessage, MinotaurAlgorithm minotaur);
    }
}
