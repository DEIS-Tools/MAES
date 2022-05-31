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

using System.Collections.Generic;

namespace Maes.Utilities {
    public class Functional {
        public delegate T Factory<T>();

        public delegate bool XBetterThanY<T>(T x, T y);

        public static T TakeBest<T>(T[] values, XBetterThanY<T> xBetterThanY) {
            var currentMin = values[0];
            for (int i = 1; i < values.Length; i++) {
                if (xBetterThanY(values[i], currentMin))
                    currentMin = values[i];
            }
            return currentMin;
        }
        
        
        public static T TakeBest<T>(List<T> values, XBetterThanY<T> xBetterThanY) {
            var currentMin = values[0];
            for (int i = 1; i < values.Count; i++) {
                if (xBetterThanY(values[i], currentMin))
                    currentMin = values[i];
            }
            return currentMin;
        }
    }
}