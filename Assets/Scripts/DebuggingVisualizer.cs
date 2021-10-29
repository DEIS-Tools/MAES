using System.Collections.Generic;
using Dora.Robot;
using UnityEngine;

namespace Dora {
    public class DebuggingVisualizer : ISimulationUnit {
        
        private Queue<CommunicationLink> _links = new Queue<CommunicationLink>();
        private Color _linkColor = new Color(50f / 255f, 120f / 255f, 255f / 255f);
        private int _currentTick = 0;

        // Environment tags
        private List<Vector2> _environmentTags = new List<Vector2>();
        private const float TagSquareSize = 0.3f;
        private readonly Vector3 _tagCubeSize = new Vector3(TagSquareSize, TagSquareSize, TagSquareSize);
        private readonly Color _tagColor = new Color(50f / 255f, 120f / 255f, 255f / 255f);

        private readonly struct CommunicationLink {
            public readonly MonaRobot RobotOne;
            public readonly MonaRobot RobotTwo;
            public readonly int StartPhysicsTick;
            public readonly int EndPhysicsTick;

            public CommunicationLink(MonaRobot robotOne, MonaRobot robotTwo, int startPhysicsTick, int endPhysicsTick) {
                RobotOne = robotOne;
                RobotTwo = robotTwo;
                this.StartPhysicsTick = startPhysicsTick;
                this.EndPhysicsTick = endPhysicsTick;
            }
        }

        // Debugging measure to visualize all environment tags
        public void AddEnvironmentTag(Vector2 position) {
            _environmentTags.Add(position);
        }

        // Debugging measure to visualize communication between robots
        public void AddCommunicationTrail(MonaRobot robot1, MonaRobot robot2) {
            _links.Enqueue(new CommunicationLink(robot1, robot2, _currentTick, _currentTick + 10));
        }

        public void Render() {
            // Render communication links between robots
            Gizmos.color = _linkColor;
            Vector3 offset = new Vector3(0.5f, 0.5f, 1);
            foreach (var link in _links)
                Gizmos.DrawLine(link.RobotOne.transform.position + offset, link.RobotTwo.transform.position + offset);

            foreach (var tagPosition in _environmentTags) {
                Gizmos.DrawCube(new Vector3(tagPosition.x, tagPosition.y, 1f), _tagCubeSize);
            }
        }

        public void PhysicsUpdate() {
            _currentTick++;
            // Remove old links
            while (_links.Count != 0 && _links.Peek().EndPhysicsTick < _currentTick)
                _links.Dequeue();
        }

        public void LogicUpdate() {
        }

        public object SaveState() {
            throw new System.NotImplementedException();
        }

        public void RestoreState(object stateInfo) {
            throw new System.NotImplementedException();
        }
    }
}