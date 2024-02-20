using Maes.Map;
using System;
using System.Collections.Generic;
using System.Numerics;

namespace Maes.ExplorationAlgorithm.Minotaur.Messages
{
    public class HeartbeatMessage : IMinotaurMessage
    {
        private readonly CoarseGrainedMap _map;
        private readonly List<Doorway> _doorways;
        private readonly Vector2 _location;

        public HeartbeatMessage(CoarseGrainedMap map, List<Doorway> doorways, Vector2 location)
        {
            _map = map;
            _doorways = doorways;
            _location = location;
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
