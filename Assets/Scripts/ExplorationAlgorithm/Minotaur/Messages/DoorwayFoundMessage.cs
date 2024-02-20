using System;

namespace Maes.ExplorationAlgorithm.Minotaur.Messages
{
    public class DoorwayFoundMessage : IMinotaurMessage
    {
        public IMinotaurMessage Combine(IMinotaurMessage otherMessage, MinotaurAlgorithm minotaur)
        {
            throw new NotImplementedException();
        }

        public IMinotaurMessage Process(MinotaurAlgorithm minotaur)
        {
            throw new NotImplementedException();
        }
    }
}
