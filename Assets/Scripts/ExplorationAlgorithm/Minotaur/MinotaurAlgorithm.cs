using Maes.Map;
using Maes.Robot;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEditor.Graphs;
using UnityEngine;

namespace Maes.ExplorationAlgorithm.Minotaur
{
    public partial class MinotaurAlgorithm : IExplorationAlgorithm
    {
        public float VisionArea;
        private IRobotController _controller;
        private RobotConstraints _robotConstraints;
        private CoarseGrainedMap _map;
        private int _seed;
        private Vector2Int _location;
        private List<Doorway> _doorways;
        private List<MinotaurAlgorithm> _minotaurs;
        private State _currentState;

        private Doorway _closestDoorway = null;

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
            switch (_currentState)
            {
                case State.Idle:
                    break;
                case State.Exploring:
                    break;
                case State.Auctioning:
                    break;
                case State.MovingToDoorway:
                    MoveToNearestUnexploredAreaWithinRoom();
                    break;
                case State.MovingToNearestUnexplored:
                    MoveThroughNearestUnexploredDoorway();
                    break;
            }
            Communicate();
            ExploringAlongEdge();
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
            //If guard clauses could be avoided, that would be great
            if (_closestDoorway == null)
            {
                int doorwayDistance = int.MaxValue;
                bool didNotPassDoorway = true;
                foreach (Doorway doorway in _doorways)
                {
                    List<Vector2Int> path = _controller.GetSlamMap().GetPath(_location, doorway.Position);
                    var distance = path.Count;
                    if (distance < doorwayDistance)
                    {
                        for (int i = 0; i < distance - 1; i++)
                        {
                            foreach (Doorway doorwayForPath in _doorways)
                            {
                                if (path[i] == doorwayForPath.Position) didNotPassDoorway = false;
                            }
                        }
                        if (didNotPassDoorway)
                        {
                            doorwayDistance = distance;
                            _closestDoorway = doorway;
                        }
                    };
                }
            }
            if (_closestDoorway == null) return;
            _controller.PathAndMoveTo(_closestDoorway.Position);
            if (_location == _closestDoorway.Position)
            {
                _currentState = State.Idle;
                _closestDoorway = null;
            }
        }

        private void MoveThroughNearestUnexploredDoorway()
        {
            //If guard clauses could be avoided, that would be great
            if (_closestDoorway == null)
            {
                int doorwayDistance = int.MaxValue;
                foreach (Doorway doorway in _doorways)
                {
                    List<Vector2Int> path = _controller.GetSlamMap().GetPath(_location, doorway.Position);
                    var distance = path.Count;
                    if (distance < doorwayDistance)
                    {
                        doorwayDistance = distance;
                        _closestDoorway = doorway;
                    }
                }
            }
            if (_closestDoorway == null) return;
            _controller.PathAndMoveTo(_closestDoorway.Position);
            if (_location == _closestDoorway.Position)
            {
                _currentState = State.Idle;
                _closestDoorway = null;
            }
        }
    }
}
