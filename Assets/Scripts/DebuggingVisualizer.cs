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
        private readonly List<EnvironmentTag> _environmentTags = new List<EnvironmentTag>();

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
        public void AddEnvironmentTag(EnvironmentTag tag) {
            _environmentTags.Add(tag);
        }

        // Debugging measure to visualize communication between robots
        public void AddCommunicationTrail(MonaRobot robot1, MonaRobot robot2) {
            _links.Enqueue(new CommunicationLink(robot1, robot2, _currentTick + 10));
            
        }

        /*public void Render() {
            // Render communication links between robots
            Gizmos.color = _linkColor;
            Vector3 offset = new Vector3(0.5f, 0.5f, 1);
            foreach (var link in _links)
                link.LineRenderer.SetPositions(new Vector3[] {link.RobotOne.transform.position + new Vector3(0,0,-1), link.RobotTwo.transform.position + new Vector3(0,0,-1)});
                // Gizmos.DrawLine(link.RobotOne.transform.position + offset, link.RobotTwo.transform.position + offset);
            
            foreach (var placedTag in _environmentTags) 
                placedTag.tag.DrawTag(placedTag.WorldPosition);
        }*/

        public void RenderVisibleTags() {
            foreach (var tag in _environmentTags) {
                tag.SetVisibility(true);
            }
        }

        public void RenderSelectedVisibleTags(int id) {
            foreach (var tag in _environmentTags) {
                tag.SetVisibility(tag.sender == id);
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

        public void HideAllTags() {
            foreach (var tag in _environmentTags) {
                tag.SetVisibility(false);
            }
        }

        public void RenderCommunicationLines() {
            foreach (var link in _links) {
                link.LineRenderer.SetPositions(new Vector3[] {link.RobotOne.transform.position + new Vector3(0, 0, -.2f), link.RobotTwo.transform.position + new Vector3(0, 0, -.2f)});
            }
        }
    }
}