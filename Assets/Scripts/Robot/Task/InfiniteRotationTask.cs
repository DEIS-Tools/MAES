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

namespace Maes.Robot.Task {
    internal class InfiniteRotationTasK : ITask {
        public float ForceMultiplier;

        public InfiniteRotationTasK(float forceMultiplier) {
            ForceMultiplier = forceMultiplier;
        }

        public MovementDirective GetNextDirective() {
            var absMultiplier = Math.Abs(ForceMultiplier);
            return ForceMultiplier < 0 ? MovementDirective.Left(absMultiplier) : MovementDirective.Right(absMultiplier);
        }

        public bool IsCompleted() {
            return false;
        }
    }
}