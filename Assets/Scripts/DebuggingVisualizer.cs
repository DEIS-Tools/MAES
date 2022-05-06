using System.Collections.Generic;
using Maes.Map;
using Maes.Robot;
using UnityEngine;

namespace Maes {
    public class DebuggingVisualizer : ISimulationUnit {
        
        private Queue<CommunicationLink> _links = new Queue<CommunicationLink>();
        private Color _linkColor = new Color(50f / 255f, 120f / 255f, 255f / 255f);
        private int _currentTick = 0;

        private bool _shouldRenderAllTags = false;

        // Environment tags
        private readonly List<EnvironmentTaggingMap.PlacedTag> _environmentTags = new List<EnvironmentTaggingMap.PlacedTag>();

        private readonly Color _tagColor = new Color(50f / 255f, 120f / 255f, 255f / 255f);
        private readonly Color _readableTagColor = new Color(255f / 255f, 150f / 255f, 255f / 255f);

        private readonly struct CommunicationLink {
            public readonly MonaRobot RobotOne;
            public readonly MonaRobot RobotTwo;
            public readonly int EndPhysicsTick;
            public readonly LineRenderer LineRenderer;

            public CommunicationLink(MonaRobot robotOne, MonaRobot robotTwo, int endPhysicsTick) {
                RobotOne = robotOne;
                RobotTwo = robotTwo;
                EndPhysicsTick = endPhysicsTick;
                LineRenderer = GameObject.Instantiate(Resources.Load<LineRenderer>("LineRendererPrefab"));
                LineRenderer.positionCount = 2;
                LineRenderer.widthMultiplier = .03f;
                LineRenderer.startColor = Color.yellow;
                LineRenderer.name = ToString();
            }

            public override string ToString() {
                return $"LineRenderer {RobotOne.id} - {RobotTwo.id}";
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
            _links.Enqueue(new CommunicationLink(robot1, robot2, _currentTick + 10));
            
        }

        public void Render() {
            // Render communication links between robots
            Gizmos.color = _linkColor;
            Vector3 offset = new Vector3(0.5f, 0.5f, 1);
            foreach (var link in _links)
                link.LineRenderer.SetPositions(new Vector3[] {link.RobotOne.transform.position + new Vector3(0,0,-1), link.RobotTwo.transform.position + new Vector3(0,0,-1)});
                // Gizmos.DrawLine(link.RobotOne.transform.position + offset, link.RobotTwo.transform.position + offset);
            
            foreach (var placedTag in _environmentTags) 
                placedTag.tag.DrawTag(placedTag.WorldPosition);
        }

        public void RenderVisibleTags() {
            foreach (var placedTag in _environmentTags) {
                if (placedTag.tag is VisibleTag tag) {
                    tag.SetVisibility(true);
                    tag.DrawTag(new Vector3(placedTag.WorldPosition.x, placedTag.WorldPosition.y ,-.1f));
                }
            }
        }

        public void RenderSelectedVisibleTags(int id) {
            foreach (var placedTag in _environmentTags) {
                if (placedTag.tag is VisibleTag tag) {
                    tag.SetVisibility(tag.sender == id);
                    tag.DrawTag(new Vector3(placedTag.WorldPosition.x, placedTag.WorldPosition.y ,-.1f));
                }
            }
        }

        public void PhysicsUpdate() {
            _currentTick++;
            // Remove old links
            while (_links.Count != 0 && _links.Peek().EndPhysicsTick < _currentTick) {
                var link = _links.Dequeue();
                Object.Destroy(GameObject.Find(link.ToString()));
            }
        }

        public void LogicUpdate() { }

        public object SaveState() {
            throw new System.NotImplementedException();
        }

        public void RestoreState(object stateInfo) {
            throw new System.NotImplementedException();
        }

        public void UnRenderAllTags() {
            foreach (var placedTag in _environmentTags) {
                if (placedTag.tag is VisibleTag tag) {
                    tag.SetVisibility(false);
                }
            }
        }

        public void RenderCommunicationLines() {
            foreach (var link in _links) {
                link.LineRenderer.SetPositions(new Vector3[] {link.RobotOne.transform.position + new Vector3(0, 0, -.2f), link.RobotTwo.transform.position + new Vector3(0, 0, -.2f)});
            }
        }
    }
}