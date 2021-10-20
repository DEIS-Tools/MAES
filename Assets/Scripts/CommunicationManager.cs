using System.Collections.Generic;
using Dora.MapGeneration;
using Dora.Robot;
using UnityEngine;

namespace Dora
{
    // Messages sent through this class will be subject to communication range and line of sight.
    // Communication is non-instantaneous. Messages will be received by other robots after one logic tick. 
    public class CommunicationManager: ISimulationUnit
    {
        private RobotConstraints _robotConstraints;

        // Messages that will sent during the next logic update
        private List<Message> _queuedMessages = new List<Message>();

        // Messages that were sent last tick and can now be read 
        private List<Message> _readableMessages = new List<Message>();

        private RayTracingMap<bool> _rayTracingMap;

        private readonly struct Message
        {
            public readonly object Contents;
            public readonly int Sender;
            public readonly Vector2 broadcastCenter;

            public Message(object contents, int sender, Vector2 broadcastCenter)
            {
                Contents = contents;
                Sender = sender;
                this.broadcastCenter = broadcastCenter;
            }
        }
        
        public CommunicationManager(SimulationMap<bool> collisionMap, RobotConstraints robotConstraints)
        {
            _robotConstraints = robotConstraints;
            _rayTracingMap = new RayTracingMap<bool>(collisionMap);
        }

        // Adds a message to the broadcast queue
        public void BroadcastMessage(MonaRobot sender, in object messageContents) {
            _queuedMessages.Add(new Message(messageContents, sender.id, sender.transform.position));
        }

        // Returns a list of messages sent by other robots
        public List<object> ReadMessages(MonaRobot receiver)
        {
            List<object> messages = new List<object>();
            Vector2 position = receiver.transform.position;
            foreach (var message in _readableMessages)
            {
                // The robot will not receive its own messages
                if (message.Sender == receiver.id) continue;
                if (CanSignalTravelBetween(message.broadcastCenter, position))
                    messages.Add(message.Contents);
            }

            return messages;
        }

        private bool CanSignalTravelBetween(Vector2 pos1, Vector2 pos2)
        {
            if (Vector2.Distance(pos1, pos2) > _robotConstraints.BroadcastRange)
                return false;

            if (!_robotConstraints.BroadcastBlockedByWalls) 
                return true; 
            
            // If walls cannot be ignored, perform a raycast to check line of sight between the given points
            var angle = Vector2.Angle(pos1, pos2);
            bool canTravel = true;
            _rayTracingMap.Raytrace(pos1, angle, _robotConstraints.BroadcastRange,
                (_, cellIsSolid) =>
                {
                    if (cellIsSolid)
                        canTravel = false;
                    return canTravel;
                });
            return canTravel;
        }

        public void LogicUpdate(SimulationConfiguration config)
        {
            // Move messages sent last tick into readable messages
            _readableMessages.Clear();
            _readableMessages.AddRange(_queuedMessages);
            _queuedMessages.Clear();
        }
        
        
        
        public void PhysicsUpdate(SimulationConfiguration config)
        {
            // No physics update needed
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