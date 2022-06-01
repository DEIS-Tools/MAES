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

using Maes.Robot;
using Maes.Robot.Task;
using Random = System.Random;

namespace Maes.ExplorationAlgorithm.RandomBallisticWalk {
    public class RandomExplorationAlgorithm : IExplorationAlgorithm {
        private IRobotController _robotController;
        private bool _hasJustRotated = true;
        private readonly Random _random;

        public RandomExplorationAlgorithm(int randomSeed) {
            _random = new Random(randomSeed);
        }

        public RandomExplorationAlgorithm(Robot2DController robotControllerController, int randomSeed) {
            _robotController = robotControllerController;
            _random = new Random(randomSeed);
        }

        public void UpdateLogic() {
            //_robotController.ReadNearbyTags();
            var status = _robotController.GetStatus();
            if (status == RobotStatus.Idle) {
                if (!_hasJustRotated) {
                    var direction = _random.Next(0, 1) == 0 ? -1 : 1;
                    var degrees = _random.Next(50, 180);
                    _robotController.Rotate(degrees * direction);
                    _hasJustRotated = true;
                }
                else {
                    _robotController.StartMoving();
                    _hasJustRotated = false;
                }
            }
        }

        public void SetController(Robot2DController controller) {
            this._robotController = controller;
        }

        public string GetDebugInfo() {
            return "";
        }
    }
}