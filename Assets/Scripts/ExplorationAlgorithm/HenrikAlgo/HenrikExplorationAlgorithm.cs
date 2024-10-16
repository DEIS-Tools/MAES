// Copyright 2024
// Contributors: Henneboy

using System.Text;
using Maes.Map;
using Maes.Robot;
using Maes.Robot.Task;
using UnityEngine;

namespace Maes.ExplorationAlgorithm.HenrikAlgo
{
    public class HenrikExplorationAlgorithm : IExplorationAlgorithm
    {
        private IRobotController _robotController;
        private Vector2Int? _targetTile = null;
        public HenrikExplorationAlgorithm()
        {
        }

        public HenrikExplorationAlgorithm(Robot2DController robotControllerController)
        {
            _robotController = robotControllerController;
        }

        public void SetController(Robot2DController controller)
        {
            this._robotController = controller;
        }

        public void UpdateLogic()
        {
            var status = _robotController.GetStatus();
            if (status == RobotStatus.Idle)
            {
                CollisionCorrector();
                _targetTile = _robotController.GetSlamMap().CoarseMap.GetNearestTileFloodFill(_robotController.GetSlamMap().CoarseMap.GetCurrentPosition(), SlamMap.SlamTileStatus.Unseen);
            }
            if (_targetTile != null)
            {
                _robotController.PathAndMoveTo(_targetTile.Value);
                if (_robotController.GetSlamMap().CoarseMap.IsTileExplored(_targetTile.Value))
                {
                    _targetTile = null;
                    _robotController.StopCurrentTask();
                    return;
                }
                return;
            }
        }

        private void CollisionCorrector()
        {
            if (_robotController.IsCurrentlyColliding())
            {
                _robotController.Move(0.2f, true);
                _robotController.Rotate(45);
                _robotController.Move(0.2f, true);
            }
        }

        public string GetDebugInfo()
        {
            var info = new StringBuilder();
            info.AppendLine($"Target: {_targetTile}.");
            info.AppendLine($"Current position: {_robotController.GetSlamMap().CoarseMap.GetCurrentPosition()}");
            info.AppendLine($"Status: {_robotController.GetStatus()}.");

            return info.ToString();
        }
    }
}