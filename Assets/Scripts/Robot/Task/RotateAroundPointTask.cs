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

using Maes.Utilities;
using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Maes.Robot.Task
{
    internal class RotateAroundPointTask : ITask
    {
        private readonly Vector2Int _point;
        private readonly float _radius;
        private readonly float _force;
        private readonly bool _counterClockwise;

        // Radius in slam
        public RotateAroundPointTask(Vector2Int point, float radius, float force, bool counterClockwise)
        {   
            _point = point;
            _radius = radius*2;
            _force = force;
            _counterClockwise = counterClockwise;
        }

        public MovementDirective GetNextDirective()
        {

            var ratioBetweenWheelForces = GetRatioFromRadius();
            // Apply smaller force on inner wheel depending on direction
            var leftForce = _counterClockwise ? _force * ratioBetweenWheelForces : _force;
            var rightForce = _counterClockwise ? _force : _force * ratioBetweenWheelForces;
            return new MovementDirective(leftForce, rightForce);
        }

        private float GetRatioFromRadius()
        {
            // Emperically found numbers dependent on a MonaRobot with 0.6 relative size
            // Data has been collated and can be found in Statistics/circle_data.xlsx
            return -1 / (0.940824360231996f * _radius + 0.502227229721352f) + 1;
        }

        public bool IsCompleted()
        {
            return false;
        }
    }
}
