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
    public class DictionaryWithDefault<TKey, TValue> : Dictionary<TKey, TValue>
    {
        TValue _default;
        public TValue DefaultValue {
            get { return _default; }
            set { _default = value; }
        }
        public DictionaryWithDefault() : base() { }
        public DictionaryWithDefault(TValue defaultValue) : base() {
            _default = defaultValue;
        }
        public new TValue this[TKey key]
        {
            get { 
                TValue t;
                return base.TryGetValue(key, out t) ? t : _default;
            }
            set { base[key] = value; }
        }
    }
}