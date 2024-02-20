using Maes.Map;
using Maes.Robot;
using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Maes.ExplorationAlgorithm.Minotaur
{
    public class MinotaurAlgorithm : IExplorationAlgorithm
    {
        public float VisionArea;
        private IRobotController _controller;
        private RobotConstraints _robotConstraints;
        private CoarseGrainedMap _map;
        private int _seed;
        private Vector2 _location;
        private List<Doorway> _doorways;
        private List<MinotaurAlgorithm> _minotaurs;
        private State _currentState;

        private enum State
        {
            Idle,
            Exploring,
            Auctioning,
            MovingToDoorway,
            MovingToNearestUnexplored
        }

        public MinotaurAlgorithm(RobotConstraints robotConstraints, int seed)
        {
            _robotConstraints = robotConstraints;
            _seed = seed;
        }

        public string GetDebugInfo()
        {
            return $"State: {Enum.GetName(typeof(State), _currentState)}" +
                   $"\nCoarse Map Position: {_map.GetApproximatePosition()}";
        }

        public void SetController(Robot2DController controller)
        {
            _controller = controller;
            _map = _controller.GetSlamMap().GetCoarseMap();
        }

        public void UpdateLogic()
        {
            Communicate();
            ExploringAlongEdge();
            MoveToNearestUnexploredAreaWithinRoom();
            MoveThroughNearestUnexploredDoorway();
        }

        private void Communicate()
        {
            throw new System.NotImplementedException();
        }

        private void ExploringAlongEdge()
        {
            DoorwayDetection();
        }

        private void DoorwayDetection()
        {
            throw new System.NotImplementedException();
        }

        private void MoveToNearestUnexploredAreaWithinRoom()
        {
            throw new System.NotImplementedException();
        }

        private void MoveThroughNearestUnexploredDoorway()
        {
            throw new System.NotImplementedException();
        }
    }
}
