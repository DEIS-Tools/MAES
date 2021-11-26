using System.Collections.Generic;
using Maes.Map;
using Maes.Robot;
using UnityEngine;

namespace Maes {
    public class DebuggingVisualizer : ISimulationUnit {
        
        private Queue<CommunicationLink> _links = new Queue<CommunicationLink>();
        private Color _linkColor = new Color(50f / 255f, 120f / 255f, 255f / 255f);
        private int _currentTick = 0;

        // Environment tags
        private readonly List<EnvironmentTaggingMap.PlacedTag> _environmentTags = new List<EnvironmentTaggingMap.PlacedTag>();

        private readonly Color _tagColor = new Color(50f / 255f, 120f / 255f, 255f / 255f);
        private readonly Color _readableTagColor = new Color(255f / 255f, 150f / 255f, 255f / 255f);

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
        public void AddEnvironmentTag(EnvironmentTaggingMap.PlacedTag tag) {
            _environmentTags.Add(tag);
        }

        public void RemoveEnvironmentTagAt(Vector2 position) {
            var tag = _environmentTags.Find(t => t.WorldPosition == position);
            _environmentTags.Remove(tag);
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
            
            foreach (var placedTag in _environmentTags) 
                placedTag.tag.DrawGizmos(placedTag.WorldPosition);
        }

        public void PhysicsUpdate() {
            _currentTick++;
            // Remove old links
            while (_links.Count != 0 && _links.Peek().EndPhysicsTick < _currentTick)
                _links.Dequeue();
        }

        public void LogicUpdate() { }

        public object SaveState() {
            throw new System.NotImplementedException();
        }

        public void RestoreState(object stateInfo) {
            throw new System.NotImplementedException();
        }
    }
}