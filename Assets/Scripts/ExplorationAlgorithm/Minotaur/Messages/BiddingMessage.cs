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
                    if ((_allBids.Count + 1) / 2 > 1)
                    {
                        var orderedBids = _allBids.OrderBy(key => key.Value);
                        var winnerbids = orderedBids.Take((_allBids.Count + 1) / 2);
                        foreach ((int key, int value) in winnerbids)
                        {
                            winnerIDList.Add(key);
                        }
                        minotaur._waypoint = new Waypoint(minotaur._map.FromSlamMapCoordinate(_doorway.Center + _doorway.ApproachedDirection.Vector * 4), Waypoint.WaypointType.Door, true);
                        minotaur._controller.PathAndMoveTo(minotaur._waypoint.Value.Destination);
                        minotaur._doorways.Find(x => x.Center == _doorway.Center).Explored = true;
                        return new AuctionResultMessage(winnerIDList, _doorway);
                    }
                    else if (_allBids.Count / 2 == 1)
                    {
                        minotaur._waypoint = new Waypoint(minotaur._map.FromSlamMapCoordinate(_doorway.Center + _doorway.ApproachedDirection.Vector * 4), Waypoint.WaypointType.Door, true);
                        minotaur._controller.PathAndMoveTo(minotaur._waypoint.Value.Destination);
                        minotaur._doorways.Find(x => x.Center == _doorway.Center).Explored = true;
                        return this;
                    }
                    else
                    {
                        return this;
                    }
                }
                else
                {
                    return this;
                }
            }
        }
    }
}
