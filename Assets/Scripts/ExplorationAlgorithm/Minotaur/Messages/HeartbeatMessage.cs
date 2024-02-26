using Maes.Map;
using System;
using System.Collections.Generic;
using System.Numerics;

namespace Maes.ExplorationAlgorithm.Minotaur
{
    public partial class MinotaurAlgorithm : IExplorationAlgorithm
    {
        public class HeartbeatMessage : IMinotaurMessage
        {
            internal SlamMap map;
            internal List<Doorway> doorways;
            internal Vector2 location;

            public HeartbeatMessage(SlamMap map, List<Doorway> doorways, Vector2 location)
            {
                this.map = map;
                this.doorways = doorways;
                this.location = location; //Consider location with robot id and a timer, for fancy decision making later
            }

            public IMinotaurMessage? Combine(IMinotaurMessage otherMessage, MinotaurAlgorithm minotaur)
            {
                if (otherMessage is HeartbeatMessage heartbeatMessage) {
                    List<SlamMap> newMap = new(){heartbeatMessage.map};
                    SlamMap.Combine(map, newMap); //layers of pass by reference, map in controller is updated with the info from message

                    foreach (Doorway doorway in heartbeatMessage.doorways) 
                    {
                        foreach (Doorway ownDoorway in doorways)
                            if (Math.Abs(doorway.Position.x - ownDoorway.Position.x) < 0.2 && Math.Abs(doorway.Position.y - ownDoorway.Position.y) < 0.2
                            && doorway.ApproachedDirection.OppositeDirection() == ownDoorway.ApproachedDirection)
                            {
                                doorway.Explored = true;
                            } else 
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
                throw new NotImplementedException(); //
            }
        }
    }
}
