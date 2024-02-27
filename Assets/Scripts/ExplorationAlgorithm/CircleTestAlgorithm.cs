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
        private bool turning = false;
        private int _ticks = 0;
        public CircleTestAlgorithm(float leftForce, float rightForce)
        {
            _leftForce = leftForce;
            _rightForce = rightForce;
        }

        public string GetDebugInfo()
        {
            return (_points.Count == 2).ToString();
        }

        public void SetController(Robot2DController controller)
        {
            _controller = controller;
            _map = _controller.GetSlamMap().GetCoarseMap();
        }

        public void UpdateLogic()
        {
            _ticks++;
            var position = _controller.SlamMap.GetCurrentPositionSlamTile();
            if (!turning)
            {
                _controller.SetWheelForceFactors(_leftForce, _rightForce);
                turning = true;
                _points.Add(position);
            }
            if (_controller._transform.position.y < 0f)
            {
                _points.Add(position);
            }

            if (_points.Count == 2)
            {
                var radius = Vector2Int.Distance(_points[0], _points[1]) / 2;
                var distanceBetweenWheels = Vector2.Distance(_controller._leftWheel.position, _controller._rightWheel.position);
                var innerRadius = radius - distanceBetweenWheels / 2;
                var outerRadius = radius + distanceBetweenWheels / 2;
                var ratioBetweenWheelSpeeds = (innerRadius / outerRadius);
                using (var file = File.AppendText("C:\\Users\\Thor Beregaard\\Desktop\\circle.csv"))
                {
                    var radText = radius.ToString(new CultureInfo("en-US"));
                    var ratioText = ratioBetweenWheelSpeeds.ToString(new CultureInfo("en-US"));
                    file.WriteLine($"{radText},{_ticks*2},{_leftForce},{_rightForce},{ratioText}");
                }
                _controller.StopCurrentTask();
            }
        }
    }
}
