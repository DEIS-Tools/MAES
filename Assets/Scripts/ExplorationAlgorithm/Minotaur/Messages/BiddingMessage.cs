using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine.Experimental.AI;
using YamlDotNet.Core.Tokens;

namespace Maes.ExplorationAlgorithm.Minotaur
{
    public partial class MinotaurAlgorithm : IExplorationAlgorithm
    {
        public class BiddingMessage : IMinotaurMessage
        {
            private readonly int _requestorRobotID;
            private Dictionary<int, int> _allBids = new();
            private readonly Doorway _doorway;

            public BiddingMessage(int requestorRobotID, Dictionary<int, int> allBids, Doorway doorway)
            {
                _requestorRobotID = requestorRobotID;
                _allBids = allBids;
                _doorway = doorway;
            }

            public IMinotaurMessage Combine(IMinotaurMessage otherMessage, MinotaurAlgorithm minotaur)
            {
                if (otherMessage is BiddingMessage biddingMessage
                && biddingMessage._doorway == _doorway
                && biddingMessage._requestorRobotID == _requestorRobotID)
                {
                    foreach ((int key, int value) in biddingMessage._allBids)
                    {
                        _allBids[key] = value; //This would overwrite bid values for the requestor and doorway, if it somehow changes, through it shouldnt
                    }
                }
                return this;
            }
            public IMinotaurMessage Process(MinotaurAlgorithm minotaur)
            {
                if (minotaur._controller.GetRobotID() == _requestorRobotID)
                {
                    List<int> winnerIDList = new();
                    if (_allBids.Count / 2 > 1)
                    {
                        var orderedBids = _allBids.OrderByDescending(key => key.Value);
                        var winnerbids = orderedBids.Take(_allBids.Count / 2);
                        foreach ((int key, int value) in winnerbids)
                        {
                            winnerIDList.Add(value);
                        }
                        minotaur._waypoint = new Waypoint(_doorway.Center, Waypoint.WaypointType.Door, true);
                        minotaur._controller.PathAndMoveTo(_doorway.Center);
                    }
                    else if (_allBids.Count / 2 == 1)
                    {
                        minotaur._waypoint = new Waypoint(_doorway.Center, Waypoint.WaypointType.Door, true);
                        minotaur._controller.PathAndMoveTo(_doorway.Center);
                    } //else pass doorway, maybe here?

                    return new AuctionResultMessage(winnerIDList, _doorway);
                }
                else
                {
                    return this;
                }
            }
        }
    }
}
