using System.Collections.Generic;
using Dora.Robot;
using UnityEngine;

namespace Dora
{
    public class DebuggingVisualizer: ISimulationUnit
    {

        private Queue<CommunicationLink> _links = new Queue<CommunicationLink>();

        private Color _linkColor = new Color(50f / 255f, 120f / 255f, 255f / 255f);
        
        private int _currentTick = 0;

        private readonly struct CommunicationLink
        {
            public readonly MonaRobot RobotOne;
            public readonly MonaRobot RobotTwo;
            public readonly int StartPhysicsTick;
            public readonly int EndPhysicsTick;

            public CommunicationLink(MonaRobot robotOne, MonaRobot robotTwo, int startPhysicsTick, int endPhysicsTick)
            {
                RobotOne = robotOne;
                RobotTwo = robotTwo;
                this.StartPhysicsTick = startPhysicsTick;
                this.EndPhysicsTick = endPhysicsTick;
            }
        }
        
        // Debugging measure to visualize communication between robots
        public void AddCommunicationTrail(MonaRobot robot1, MonaRobot robot2)
        {
            _links.Enqueue(new CommunicationLink(robot1, robot2, _currentTick, _currentTick + 30));
        }

        public void Render()
        {
            // Render communication links between robots
            Gizmos.color = _linkColor;
            Vector3 offset = new Vector3(0.5f,0.5f, 1);
            foreach (var link in _links)
                Gizmos.DrawLine(link.RobotOne.transform.position + offset, link.RobotTwo.transform.position + offset);
        }

        public void PhysicsUpdate(SimulationConfiguration config)
        {
            _currentTick++;
            // Remove old links
            while (_links.Count != 0 && _links.Peek().EndPhysicsTick < _currentTick)
                _links.Dequeue();
        }
        
        public void LogicUpdate(SimulationConfiguration config)
        {
            
        }

        public object SaveState()
        {
            throw new System.NotImplementedException();
        }

        public void RestoreState(object stateInfo)
        {
            throw new System.NotImplementedException();
        }
        
    }
}