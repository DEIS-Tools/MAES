// Copyright 2022 MAES
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
// Contributors: Malte Z. Andreasen, Philip I. Holler and Magnus K. Jensen
// 
// Original repository: https://github.com/MalteZA/MAES

namespace Maes.Map.MapGen {
    internal static class TileTypes {
        internal const int ROOM_TYPE = 0, HALL_TYPE = 1, WALL_TYPE = 2, CONCRETE_WALL_TYPE = 3, WOOD_WALL_TYPE = 4, METAL_WALL_TYPE = 5;
        /* attenuation:
         Concrete = 8-15 dB | 2.4GHz/1.3GHz
         Wood     =    3 dB | 2.4GHz
         Metal    =   26 dB | 815MHz
         */
    }
}