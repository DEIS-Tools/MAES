using System.Collections;
using System.Collections.Generic;
using UnityEditor.PackageManager;
using UnityEngine;

namespace Maes.Robot.Task
{
    internal class RotateAroundPointTask : ITask
    {
        private readonly float _radius;
        private readonly float _distanceBetweenWheels;
        private readonly float _force;
        private readonly bool _counterClockwise;

        public RotateAroundPointTask(float radius, float distanceBetweenWheels, float force, bool counterClockwise) 
        {
            _radius = radius;
            _distanceBetweenWheels = distanceBetweenWheels;
            _force = force;
            _counterClockwise = counterClockwise;
        }

        public MovementDirective GetNextDirective()
        {
            var innerRadius = _radius - _distanceBetweenWheels/2;
            var outerRadius = _radius + _distanceBetweenWheels/2;
            var ratioBetweenWheelSpeeds = (innerRadius / outerRadius)/1.35f; // Sorry for magic number
            Debug.Log($"inner: {innerRadius}, outer: {outerRadius}");

            var leftForce = _counterClockwise ? _force * ratioBetweenWheelSpeeds : -_force;
            var rightForce = _counterClockwise ? -_force : _force * ratioBetweenWheelSpeeds;
            Debug.Log($"left: {leftForce}, right: {rightForce}");
            return new MovementDirective(1, 0);
        }

        public bool IsCompleted()
        {
            return false;
        }
    }
}
