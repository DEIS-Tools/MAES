using System;
using System.Collections.Generic;

namespace Maes.ExplorationAlgorithm.Minotaur
{
    public partial class MinotaurAlgorithm : IExplorationAlgorithm
    {
        public class AuctionResultMessage : IMinotaurMessage
        {
            private readonly List<int> _winnerList = new();
            private readonly Doorway _doorway;

            public AuctionResultMessage(List<int> winners, Doorway doorway)
            {
                _winnerList = winners;
                _doorway = doorway;
            }
            public IMinotaurMessage Combine(IMinotaurMessage otherMessage, MinotaurAlgorithm minotaur)
            {
                return this;
            }

            public IMinotaurMessage? Process(MinotaurAlgorithm minotaur)
            {
                // TODO: Should move to doorway that auction was won for, if in winner list
                if (_winnerList.Contains(minotaur._controller.GetRobotID()))
                {
                    minotaur._controller.StopCurrentTask();
                    minotaur._waypoint = new Waypoint(minotaur._map.FromSlamMapCoordinate(_doorway.Center + _doorway.ApproachedDirection.Vector * 4), Waypoint.WaypointType.NearestDoor, true);
                    minotaur._controller.PathAndMoveTo(minotaur._waypoint.Value.Destination);
                    return this;
                }
                return null;
            }
        }
    }
}