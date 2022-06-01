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

using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Maes.Map;
using Maes.Robot;
using Maes.Robot.Task;
using Maes.Utilities;
using Maes.YamlConfig;
using RosMessageTypes.BuiltinInterfaces;
using RosMessageTypes.Geometry;
using RosMessageTypes.Maes;
using RosMessageTypes.Rosgraph;
using RosMessageTypes.Sensor;
using RosMessageTypes.Std;
using Unity.Robotics.Core;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;
using UnityEngine.PlayerLoop;
using UnityEngine.UIElements;

namespace Maes.ExplorationAlgorithm {
    internal class Ros2Algorithm : IExplorationAlgorithm {
        private Robot2DController _controller;
        private ROSConnection _ros;
        private string _robotRosId; // e.g. robot0
        private string _topicPrefix; // Is set in SetController method, e.g. /robot0
        private string _stateTopic = "/maes_state";
        private string _broadcastTopic = "/maes_broadcast";
        private string _depositTagTopic = "/maes_deposit_tag";
        private string _cmdVelTopic = "/cmd_vel";
        private Transform _worldPosition; // Used for finding position of relative objects and sending it to ROS

        private int _tick = 0;
        
        // Used to react to cmlVel from ROS
        private float _rosLinearSpeed = 0f;
        private float _rosRotationSpeed = 0f;

        // Service calls from ROS uses a callback function. We need to store the results 
        // and act on them in the next logic tick
        private List<string> _msgsToBroadcast = new List<string>();
        private List<string> _envTagsToDeposit = new List<string>();

        public void UpdateLogic() {
            ReactToCmdVel(_rosLinearSpeed, _rosRotationSpeed);

            if(_envTagsToDeposit.Count != 0)
                ReactToBroadcastRequests();
            if(_msgsToBroadcast.Count != 0)
                ReactToDepositTagRequests();

            PublishState();
            
            _tick++;
        }
        
        private void ReactToDepositTagRequests() {
            foreach (var tagMsg in _envTagsToDeposit) {
                _controller.DepositTag(tagMsg);
            }
            _envTagsToDeposit.Clear();
        }

        private void ReactToBroadcastRequests() {
            foreach (var msg in _msgsToBroadcast) {
                _controller.Broadcast(new RosBroadcastMsg(msg, _robotRosId));
            }
            _msgsToBroadcast.Clear();
        }

        private void PublishState() {
            var state = new StateMsg();
            var robotPosition = new Vector2(_worldPosition.position.x, _worldPosition.position.y);
            var robot_rotation = _worldPosition.rotation.eulerAngles.z - 90f;
            // Flip signs like also done in TransformTreePublisher 
            // TODO: Maybe create utility function for transforming coordinates between ROS and Maes ? - Philip
            robotPosition = Geometry.ToROSCoord(robotPosition);
            // ---- tick ---- //
            state.tick = _tick;
            // ---- Status ---- //
            state.status = Enum.GetName(typeof(RobotStatus), _controller.GetStatus());
            
            // ---- Collision ---- //
            state.colliding = _controller.IsCurrentlyColliding();
            
            // ---- Incoming broadcast messages ---- //
            var objectsReceived = _controller.ReceiveBroadcast();
            var msgsReceived = objectsReceived.Cast<RosBroadcastMsg>().ToList();
            var broadcastMsgs = msgsReceived.Select(e => new BroadcastMsg(e.msg, e.sender));
            state.incoming_broadcast_msgs = broadcastMsgs.ToArray();
            
            // ---- Nearby Robots ---- //
            var nearbyRobots = _controller.SenseNearbyRobots();
            var globalAngle = _controller.GetGlobalAngle();
            // Map to relative positions of other robots
            var otherRobots = nearbyRobots.Select(e => (e.item, e.GetRelativePosition(robotPosition, robot_rotation)));
            // Convert to ros messages
            var nearbyRobotMsgs = otherRobots.Select(e =>
                new NearbyRobotMsg(e.item.ToString(), new Vector2DMsg(e.Item2.x, e.Item2.y)));
            state.nearby_robots = nearbyRobotMsgs.ToArray();

            // ---- Nearby environment tags ---- //
            var tags = _controller.ReadNearbyTags();
            var rosTagsWithPos = tags.Select(e => (e.Item.Content, GetRelativePosition(robotPosition, robot_rotation, e)));
            var rosTagAsMsgs =
                    rosTagsWithPos.Select(e => new EnvironmentTagMsg(e.Content, new Vector2DMsg(e.Item2.x, e.Item2.y)));
            state.tags_nearby = rosTagAsMsgs.ToArray();

            // ---- Publish to ROS ---- //
            _ros.Publish(_topicPrefix + _stateTopic, state);
        }

        void ReactToCmdVel(float speedCommandValue, float rotationCommandValue) {
            // Debug.Log($"{this._robotRosId} command velocities: [{speedCommandValue}, {rotationCommandValue}]");
            var robotStatus = _controller.GetStatus();
            
            if (Math.Abs(speedCommandValue) < 0.01f && Math.Abs(rotationCommandValue) > 0.0) {
                if (robotStatus != RobotStatus.Idle && !_controller.IsRotatingIndefinitely()) {
                    // The robot is currently performing another task - Stop that task and continue
                    _controller.StopCurrentTask();
                } else {
                    var sign = rotationCommandValue > 0 ? -1 : 1;
                    var force = Mathf.Pow(1.3f * rotationCommandValue, 2.0f) * 0.6f;
                    force = Mathf.Min(1f, force); // Ensure maximum force of 1.0 
                    force = sign * force; // Apply direction / sign +-
                    _controller.RotateAtRate(force);
                }
            } else if (speedCommandValue > 0) {
                if (robotStatus != RobotStatus.Idle && !_controller.IsPerformingDifferentialDriveTask()) {
                    // The robot must stop current task before starting the desired movement task
                    _controller.StopCurrentTask();
                } else {
                    // The force applied at each wheel before factoring in rotation
                    float flatWheelForce = speedCommandValue;
                    
                    // The difference in applied force between the right and left wheel. 
                    float rotationSign = rotationCommandValue > 0 ? -1 : 1;
                    float wheelForceDelta = rotationSign * Mathf.Pow(1.3f * rotationCommandValue, 2.0f) * 0.6f;
                    
                    // Calculate the force applied to each wheel and send the values to the controller
                    float leftWheelForce = flatWheelForce + wheelForceDelta / 2f;
                    float rightWheelForce = flatWheelForce - wheelForceDelta / 2f;
                    _controller.SetWheelForceFactors(leftWheelForce, rightWheelForce);
                }
            } else if (_controller.GetStatus() != RobotStatus.Idle){
                // If cmd_vel does not indicate any desired movement - then stop robot if currently moving 
                // Debug.Log("Stopping movement!");
                _controller.StopCurrentTask();    
            }
        }

        void ReceiveRosCmd(TwistMsg cmdVel) {
            _rosLinearSpeed = (float)cmdVel.linear.x;
            _rosRotationSpeed = (float)cmdVel.angular.z;
            // Debug.Log($"Robot {_controller.GetRobotID()}: Received cmdVel twist: {cmdVel.ToString()}");
        }

        public string GetDebugInfo() {
            var info = new StringBuilder();
            
            var robotPosition = new Vector2(-_worldPosition.position.x, -_worldPosition.position.y);

            info.AppendLine($"Robot ID: {this._robotRosId}");
            info.AppendLine($"Namespace: {this._topicPrefix}");
            info.AppendLine($"Position: ({robotPosition.x},{robotPosition.y})");
            info.AppendLine($"Status: {Enum.GetName(typeof(RobotStatus), _controller.GetStatus())}");
            info.AppendLine($"Is Colliding: {this._controller.IsCurrentlyColliding()}");
            info.AppendLine($"Number of nearby robots: {_controller.SenseNearbyRobots().Count}");
            info.AppendLine($"Number Incoming broadcast msg: {_controller.ReceiveBroadcast().Count}");
            info.AppendLine($"Number of nearby env tags: {_controller.ReadNearbyTags().Count}");
            info.AppendLine($"rosLinearSpeed: {this._rosLinearSpeed}");
            info.AppendLine($"rosRotationalSpeed: {this._rosRotationSpeed}");

            return info.ToString();
        }
        
        public void SetController(Robot2DController controller) {
            this._controller = controller;
            this._ros = ROSConnection.GetOrCreateInstance();
            this._topicPrefix = $"/robot{_controller.GetRobotID()}";
            this._robotRosId = $"robot{_controller.GetRobotID()}";
            
            this._worldPosition = GameObject.Find($"robot{_controller.GetRobotID()}").transform;

            // Register state publisher
            this._ros.RegisterPublisher<StateMsg>(_topicPrefix + _stateTopic);
            
            // Register broadcast and deposit tag services
            this._ros.ImplementService<BroadcastToAllRequest, BroadcastToAllResponse>(_topicPrefix + _broadcastTopic, BroadcastMessage);
            this._ros.ImplementService<DepositTagRequest, DepositTagResponse>(_topicPrefix + _depositTagTopic, DepositTag);
            
            // Subscribe to cmdVel from Nav2
            this._ros.Subscribe<TwistMsg>(_topicPrefix + _cmdVelTopic, ReceiveRosCmd);
        }

        private DepositTagResponse DepositTag(DepositTagRequest req) {
            _controller.DepositTag(req.msg);
            return new DepositTagResponse(true);
        }

        private BroadcastToAllResponse BroadcastMessage(BroadcastToAllRequest req) {
            _controller.Broadcast(new RosBroadcastMsg(req.msg, _robotRosId));
            return new BroadcastToAllResponse(true);
        }

        private class RosBroadcastMsg {
            public string msg;
            public string sender;

            public RosBroadcastMsg(string msg, string sender) {
                this.msg = msg;
                this.sender = sender;
            }
        }
        
        private Vector2 GetRelativePosition<T>(Vector2 myPosition, float globalAngle, RelativeObject<T> o) {
            var x = myPosition.x + (o.Distance * Mathf.Cos(Mathf.Deg2Rad * ((o.RelativeAngle + globalAngle) % 360)));
            var y = myPosition.y + (o.Distance * Mathf.Sin(Mathf.Deg2Rad * ((o.RelativeAngle + globalAngle) % 360)));
            return new Vector2(x, y);
        }

        public object SaveState() {
            return null;
        }

        public void RestoreState(object stateInfo) {
            return;
        }
    }
}