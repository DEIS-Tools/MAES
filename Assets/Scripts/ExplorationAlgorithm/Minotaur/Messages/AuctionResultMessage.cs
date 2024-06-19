// Copyright 2024 MAES
// 
// This file is part of MAES
// 
// MAES is free software: you can redistribute it and/or modify it under
// the terms of the GNU General Public License as published by the
// Free Software Foundation, either version 3 of the License, or (at your option)
// any later version.
// 
// MAES is distributed in the hope that it will be useful, but WITHOUT
// ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
// or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
// Public License for more details.
// 
// You should have received a copy of the GNU General Public License along
// with MAES. If not, see http://www.gnu.org/licenses/.
// 
// Contributors: Rasmus Borrisholt Schmidt, Andreas Sebastian Sørensen, Thor Beregaard
// 
// Original repository: https://github.com/Molitany/MAES

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
                    minotaur._waypoint = new Waypoint(minotaur._map.FromSlamMapCoordinate(_doorway.Center + _doorway.ExitDirection.Vector * 4), Waypoint.WaypointType.NearestDoor, true);
                    minotaur._controller.PathAndMoveTo(minotaur._waypoint.Value.Destination);
                    return this;
                }
                return null;
            }
        }
    }
}