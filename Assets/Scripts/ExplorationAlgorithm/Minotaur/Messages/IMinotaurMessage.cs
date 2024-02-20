namespace Maes.ExplorationAlgorithm.Minotaur.Messages
{
    public interface IMinotaurMessage
    {
        public IMinotaurMessage Process(MinotaurAlgorithm minotaur);
        public IMinotaurMessage Combine(IMinotaurMessage otherMessage ,MinotaurAlgorithm minotaur);
    }
}
