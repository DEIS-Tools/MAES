using System;
using System.Collections.Generic;

namespace Maes.ExplorationAlgorithm.Minotaur.Messages
{
    public class BiddingMessage : IMinotaurMessage
    {
        private readonly int RequestorRobotID;
        private Dictionary<Doorway, List<float>> _allBids = new Dictionary<Doorway, List<float>>();

        public BiddingMessage(int requestorRobotID, Dictionary<Doorway, List<float>> allBids)
        {
            RequestorRobotID = requestorRobotID;
            _allBids = allBids;
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
