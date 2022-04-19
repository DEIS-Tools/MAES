using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading.Tasks;
using Maes.Map;
using Maes.Robot;
using Maes.Robot.Task;
using RosMessageTypes.BuiltinInterfaces;
using RosMessageTypes.Geometry;
using RosMessageTypes.Maes;
using RosMessageTypes.Rosgraph;
using RosMessageTypes.Sensor;
using RosMessageTypes.Std;
using Unity.Robotics.Core;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using UnityEngine;
using UnityEngine.PlayerLoop;
using UnityEngine.UIElements;

namespace Maes.ExplorationAlgorithm {
    public class Ros2Algorithm : IExplorationAlgorithm {
        private Robot2DController _controller;
        private ROSConnection _ros;
        private string _topicPrefix; // Is set in SetController method, e.g. /robot0
        private string _stateTopic = "/maes_state";
        private string _broadcastTopic = "/maes_broadcast";
        private string _depositTagTopic = "/maes_deposit_tag";
        private string _cmdVelTopic = "/cmd_vel";

        private int _tick = 0;
        
        private float rosLinearSpeed = 0f;
        private float rosRotationSpeed = 0f;

        public void UpdateLogic() {
            if (_controller.GetStatus() == RobotStatus.Idle) {
                ReactToCmdVel(rosLinearSpeed, rosRotationSpeed);
            }
            PublishState();
            

            _tick++;
        }

        private void PublishState() {
            var state = new StateMsg();
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
            var robotPosition = _controller.SlamMap.CoarseMap.GetApproximatePosition();
            // Map to relative positions of other robots
            var otherRobots = nearbyRobots.Select(e => (e.item, e.GetRelativePosition(robotPosition, globalAngle)));
            // Convert to ros messages
            var nearbyRobotMsgs = otherRobots.Select(e =>
                new NearbyRobotMsg(e.item.ToString(), new Vector2DMsg(e.Item2.x, e.Item2.y)));
            state.nearby_robots = nearbyRobotMsgs.ToArray();
            
            // ---- Nearby environment tags ---- //
            var tags = _controller.ReadNearbyTags();
            var rosTags = tags.Cast<RelativeObject<RosTag>>();
            var rosTagsWithPos = rosTags.Select(e => (e.Item.msg, GetRelativePosition(robotPosition, globalAngle, e)));
            var rosTagAsMsgs =
                rosTagsWithPos.Select(e => new EnvironmentTagMsg(e.msg, new Vector2DMsg(e.Item2.x, e.Item2.y)));
            state.tags_nearby = rosTagAsMsgs.ToArray();
        }
        
        

        void ReactToCmdVel(float speed, float rotSpeed) {
            // We prioritise rotation over movement
            if (Math.Abs(rotSpeed) > 0.01) {
                var degrees = 10 * rotSpeed;
                Debug.Log($"Turning {degrees} degrees");
                _controller.Rotate(degrees);
            } else if (rosLinearSpeed > 0) {
                var distanceInMeters = Mathf.Min(0.4f * speed, 0.2f);
                // Debug.Log($"Moving forward in meters {distanceInMeters}");
                _controller.Move(distanceInMeters);
            }
        }

        void ReceiveRosCmd(TwistMsg cmdVel) {
            rosLinearSpeed = (float)cmdVel.linear.x;
            rosRotationSpeed = (float)cmdVel.angular.z;
            Debug.Log($"Robot {_controller.GetRobotID()}: Received cmdVel twist: {cmdVel.ToString()}");
        }
        //     ReactToCmdVel(rosLinearSpeed, rosRotationSpeed);
        //     
        //
        //     _tick++;
        // }
        //
        // void ReactToCmdVel(float speedCommandValue, float rotationCommandValue) {
        //     Debug.Log($"Command velocities: [{speedCommandValue}, {rotationCommandValue}]");
        //     var robotStatus = _controller.GetStatus();
        //     // We prioritise rotation over movement
        //     if (Math.Abs(rotationCommandValue) > 0.01) {
        //         if (robotStatus != RobotStatus.Idle && !_controller.IsRotating()) {
        //             _controller.StopCurrentTask();
        //         } else if (robotStatus == RobotStatus.Idle) {
        //             _controller.StartRotating();
        //         }
        //         // var degrees = 10 * rotationCommandValue;
        //         // Debug.Log($"Turning {degrees} degrees");
        //         // _controller.Rotate(degrees);
        //     } else if (rosLinearSpeed > 0) {
        //         if (robotStatus != RobotStatus.Idle && _controller.IsRotating()) {
        //             _controller.StopCurrentTask();
        //         } else if (robotStatus == RobotStatus.Idle) {
        //             _controller.StartMoving();
        //         }
        //         // var distanceInMeters = Mathf.Min(0.4f * speedCommandValue, 0.2f);
        //         // Debug.Log($"Moving forward in meters {distanceInMeters}");
        //         // _controller.StartMoving(); //.Move(distanceInMeters);
        //     } else if (_controller.GetStatus() != RobotStatus.Idle){
        //         Debug.Log("Stopping movement!");
        //         _controller.StopCurrentTask();    
        //     }
        //     
        // }

        public string GetDebugInfo() {
            return "";
        }
        
        public void SetController(Robot2DController controller) {
            this._controller = controller;
            this._ros = ROSConnection.GetOrCreateInstance();
            this._topicPrefix = $"/robot{_controller.GetRobotID()}";
            
            // Register state publisher
            this._ros.RegisterPublisher<StateMsg>(_topicPrefix + _stateTopic);
            
            // Register broadcast and deposit tag services
            this._ros.ImplementService<BroadcastRequest, BroadcastResponse>(_topicPrefix + _broadcastTopic, BroadcastMessage);
            this._ros.ImplementService<DepositTagRequest, DepositTagResponse>(_topicPrefix + _depositTagTopic, DepositTag);
            
            // Subscribe to cmdVel from Nav2
            this._ros.Subscribe<TwistMsg>(_topicPrefix + _cmdVelTopic, ReceiveRosCmd);
        }

        private DepositTagResponse DepositTag(DepositTagRequest req) {
            _controller.DepositTag(new RosTag(req.msg));
            return new DepositTagResponse(true);
        }

        private BroadcastResponse BroadcastMessage(BroadcastRequest req) {
            _controller.Broadcast(req.msg);
            return new BroadcastResponse(true);
        }

        private class RosBroadcastMsg {
            public string msg;
            public string sender;
        }
        
        private Vector2 GetRelativePosition<T>(Vector2 myPosition, float globalAngle, RelativeObject<T> o) {
            var x = myPosition.x + (o.Distance * Mathf.Cos(Mathf.Deg2Rad * ((o.RelativeAngle + globalAngle) % 360)));
            var y = myPosition.y + (o.Distance * Mathf.Sin(Mathf.Deg2Rad * ((o.RelativeAngle + globalAngle) % 360)));
            return new Vector2(x, y);
        }

        private class RosTag : EnvironmentTaggingMap.ITag {
            public string msg;

            public RosTag(string msg) {
                this.msg = msg;
            }
            
            private const float TagSquareSize = 0.3f;
            private readonly Vector3 _tagCubeSize = new Vector3(TagSquareSize, TagSquareSize, TagSquareSize);
            public void DrawGizmos(Vector3 position) {
                Gizmos.color = Color.magenta;
                Gizmos.DrawCube(new Vector3(position.x, position.y, -_tagCubeSize.z / 2f), _tagCubeSize);
            }
        }

        public object SaveState() {
            return null;
        }

        public void RestoreState(object stateInfo) {
            return;
        }
    }
}