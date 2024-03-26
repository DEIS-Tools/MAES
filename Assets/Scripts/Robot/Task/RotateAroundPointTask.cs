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
