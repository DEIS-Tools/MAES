// Copyright 2024
// Contributors: Henneboy

using System;
using System.Collections.Generic;
using System.Text;
using Codice.Client.Commands;
using Maes.Map;
using Maes.Robot;
using Maes.Robot.Task;
using UnityEngine;
using Random = System.Random;

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
                if (_robotController.IsCurrentlyColliding()){
                    _robotController.Move(0.5f, true);
                }
                _targetTile = _robotController.GetSlamMap().CoarseMap.GetNearestTileFloodFill(_robotController.GetSlamMap().CoarseMap.GetCurrentPosition(), SlamMap.SlamTileStatus.Unseen);
            }
            if (_targetTile != null){
                if (_robotController.HasCollidedSinceLastLogicTick()){
                    if (_robotController.GetSlamMap().CoarseMap.IsSolid(_targetTile.Value)){
                        _targetTile = null;
                        _robotController.StopCurrentTask();
                        return;
                    }
                }
                // Robot reached destination
                if (_robotController.GetSlamMap().CoarseMap.GetCurrentPosition() == _targetTile.Value){
                    _targetTile = null;
                    _robotController.StopCurrentTask();

                    return;
                }
                _robotController.PathAndMoveTo(_targetTile.Value);
                return;

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