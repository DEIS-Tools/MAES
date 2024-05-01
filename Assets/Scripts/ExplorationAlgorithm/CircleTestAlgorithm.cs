using Maes.ExplorationAlgorithm;
using Maes.Map;
using Maes.Robot;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using UnityEngine;

namespace ExplorationAlgorithm
{
    internal class CircleTestAlgorithm : IExplorationAlgorithm
    {
        private Robot2DController _controller;
        private RobotConstraints _robotConstraints;
        private CoarseGrainedMap _map;
        private List<Vector2Int> _points = new();
        private float _leftForce, _rightForce;
        private readonly float _size;
        private bool turning = false;
        private int _ticks = 0;
        private bool _shouldEnd = false;

        public CircleTestAlgorithm(float leftForce, float rightForce, float size)
        {
            _leftForce = leftForce;
            _rightForce = rightForce;
            _size = size;
        }

        public string GetDebugInfo()
        {
            // This is used to end the simulation
            return _shouldEnd.ToString();
        }

        public void SetController(Robot2DController controller)
        {
            _controller = controller;
            _map = _controller.GetSlamMap().GetCoarseMap();
        }

        public void UpdateLogic()
        {
            // Don't collect data if dragging against the wall
            if (_controller.IsCurrentlyColliding())
            {
                _controller.StopCurrentTask();
                _shouldEnd = true;
            }

            _ticks++;
            var position = _controller.SlamMap.GetCurrentPosition();
            // Simple state machine to not set a new task constantly
            if (!turning)
            {
                Debug.Log($"left: {_leftForce}, right: {_rightForce}");
                _controller.SetWheelForceFactors(_leftForce, _rightForce);
                turning = true;
                _points.Add(position);
            }
            // Add a point if we have created a half circle assuming it starts at y=0
            if (_controller.Transform.position.y < 0) 
            {
                _points.Add(position);
            }
            // Store circle in csv when we have the start point and the half circle point
            if (_points.Count == 2)
            {
                var radius = Vector2Int.Distance(_points[0], _points[1]) / 2;
                var distanceBetweenWheels = Vector2.Distance(_controller.LeftWheel.position, _controller.RightWheel.position);
                var innerRadius = radius - distanceBetweenWheels / 2;
                var outerRadius = radius + distanceBetweenWheels / 2;
                var ratioBetweenRadii = innerRadius / outerRadius;
                if (!File.Exists($@"circle_data.csv"))
                {
                    using (var file = File.AppendText($@"circle_data.csv"))
                    {
                        file.WriteLine($"radius;ticks;leftForce;rightForce;distance;ratio");
                    }
                }

                using (var file = File.AppendText($@"circle_data.csv"))
                {
                    file.WriteLine($"{radius};{_ticks * 2};{_leftForce};{_rightForce};{distanceBetweenWheels};{ratioBetweenRadii}");
                }
                _controller.StopCurrentTask();
                _shouldEnd = true;
            }
        }
    }
}
