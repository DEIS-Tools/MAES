using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using Maes.Map;
using Maes.Map.PathFinding;
using Maes.Robot;
using Maes.Robot.Task;
using Maes.Utilities;
using UnityEngine;

namespace Maes.ExplorationAlgorithm.Movement
{
    public class MovementTestAlgorithm : IExplorationAlgorithm
    {
        private Robot2DController _controller;
        private RobotConstraints _robotConstraints;
        private CoarseGrainedMap _map;
        private int _ticks = 0;
        private Vector2Int _targetTile;
        public MovementTestAlgorithm(Vector2Int targetTile)
        {
            _targetTile = targetTile;
        }

        public string GetDebugInfo()
        {
            return _controller.GetStatus().ToString();
        }

        public void SetController(Robot2DController controller)
        {
            _controller = controller;
            _map = _controller.GetSlamMap().GetCoarseMap();
        }

        public void UpdateLogic()
        {
            _controller.MoveTo(_targetTile);
        }
    }
}