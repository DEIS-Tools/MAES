using Maes.Map;
using System;
using System.Collections.Generic;
using System.Numerics;
using UnityEngine;
using System.Linq;

namespace Maes.ExplorationAlgorithm.Minotaur
{
    public partial class MinotaurAlgorithm : IExplorationAlgorithm
    {
        public class HeartbeatMessage : IMinotaurMessage
        {
            private readonly int ID;
            internal SlamMap map;
            internal List<Doorway> doorways;
            internal Vector2Int location;
            private readonly HashSet<Vector2Int> previousIntersections;

            public HeartbeatMessage(int ID, SlamMap map, List<Doorway> doorways, Vector2Int location, HashSet<Vector2Int> previousIntersections)
            {
                this.ID = ID;
                this.map = map;
                this.doorways = doorways;
                this.location = location; //Consider location with robot id and a timer, for fancy decision making later
                this.previousIntersections = previousIntersections;
            }

            public IMinotaurMessage? Combine(IMinotaurMessage otherMessage, MinotaurAlgorithm minotaur)
            {
                if (otherMessage is HeartbeatMessage heartbeatMessage)
                {
                    minotaur._otherRobotPositions[heartbeatMessage.ID] = (heartbeatMessage.location, minotaur._waypoint.HasValue ? minotaur._waypoint.Value.Destination : null);
                    minotaur._previousIntersections.UnionWith(heartbeatMessage.previousIntersections);
                    List<SlamMap> maps = new() { heartbeatMessage.map, map };
                    SlamMap.Synchronize(maps); //layers of pass by reference, map in controller is updated with the info from message

                    var amount = heartbeatMessage.doorways.Count;
                    for (int i = 0; i < amount; i++)
                    {
                        var doorway = heartbeatMessage.doorways[i];
                        if (doorways.Contains(doorway))
                        {
                            if (doorway.ApproachedDirection.OppositeDirection() == doorways.First(ownDoorway => ownDoorway.Equals(doorway)).ApproachedDirection)
                            {
                                doorways.First(ownDoorway => ownDoorway.Equals(doorway)).Explored = true;
                            }
                        }
                        else
                        {
                            doorways.Add(doorway);
                        }
                    }
                    return this;
                }
                return null;
            }

            public IMinotaurMessage Process(MinotaurAlgorithm minotaur) //Combine all, then process, but not really anything to process for heartbeat
            {
                return this;
            }
        }
    }
}

