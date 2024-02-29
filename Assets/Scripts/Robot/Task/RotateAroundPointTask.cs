using System;
using System.Collections;
using System.Collections.Generic;
using UnityEditor.PackageManager;
using UnityEngine;

namespace Maes.Robot.Task
{
    internal class RotateAroundPointTask : ITask
    {
        private readonly float _radius;
        private readonly float _force;
        private readonly bool _counterClockwise;

        public RotateAroundPointTask(float radius, float force, bool counterClockwise)
        {
            _radius = radius;
            _force = force;
            _counterClockwise = counterClockwise;
        }

        public MovementDirective GetNextDirective()
        {
            var ratioBetweenWheelForces = GetRatioFromRadius();
            var leftForce = _counterClockwise ? _force * ratioBetweenWheelForces : _force;
            var rightForce = _counterClockwise ? _force : _force * ratioBetweenWheelForces;
            return new MovementDirective(leftForce, rightForce);
        }

        private float GetRatioFromRadius()
        {
            return 0.00006f * Mathf.Pow(_radius, 3) - 0.0056f * Mathf.Pow(_radius, 2) + 0.3472f * _radius;
        }

        public bool IsCompleted()
        {
            return false;
        }
    }
}
