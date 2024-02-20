using System;

namespace Maes.ExplorationAlgorithm.Minotaur.Messages
{
    public class DoorwayFoundMessage : IMinotaurMessage
    {
        private readonly Doorway _doorway;

        public DoorwayFoundMessage(Doorway doorway)
        {
            _doorway = doorway;
        }

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
