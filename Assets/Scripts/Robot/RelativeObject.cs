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

using System;

namespace Maes.Robot {
    // Represents a relative position to some object of type T
    public class RelativeObject<T> {

        public readonly float Distance;
        public readonly float RelativeAngle;
        public T Item;

        public RelativeObject(float distance, float relativeAngle, T item) {
            Distance = distance;
            RelativeAngle = relativeAngle;
            Item = item;
        }

        public RelativeObject<V> Map<V>(Func<T, V> transformer) {
            return new RelativeObject<V>(Distance, RelativeAngle, transformer(Item));
        }

        public override string ToString() {
            return $"Relative Object {nameof(Distance)}: {Distance}, {nameof(RelativeAngle)}: {RelativeAngle}, {nameof(Item)}: {Item}";
        }
    }
}