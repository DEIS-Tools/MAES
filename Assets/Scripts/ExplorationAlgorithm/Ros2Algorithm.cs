using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading.Tasks;
using Maes.Robot;
using Maes.Robot.Task;
using RosMessageTypes.BuiltinInterfaces;
using RosMessageTypes.Geometry;
using RosMessageTypes.MaesInterface;
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
        private string _stateTopicName = "robot1/state_msg";
        private string _broadcastServiceTopicName = "robot1/broadcast_srv";
        private string _rosRobotActionTopicName = "robot1/move_action/goal";
        private double _moveDistance = -1f;

        private string _cmdVelocityTopic = "/cmd_vel";
        private string _rayTraceTopic = "/scan";
        private int _tick = 0;
        
        private float rosLinearSpeed = 0f;
        private float rosRotationSpeed = 0f;

        public void UpdateLogic() {
            var position = _controller.GetSlamMap().GetApproxPosition();
            var state = new RobotStateMsg {
                pos = new RobotPosMsg(position.x, position.y, _controller.GetGlobalAngle()),
                brdcst_msgs = new[] { new RobotMsgMsg("From other robot", 1) },
                env_tags = new[] { new EnvironmentTagMsg(10, 10, "Content of env tag") },
                has_collided = _controller.HasCollidedSinceLastLogicTick(),
                nearby_robots = new NearbyRobotMsg[]{new NearbyRobotMsg(10, 15, 1)},
                new_slam_tiles = new[] { new SlamTileMsg(new Vector2DMsg(1, 1), false) },
                status = _controller.GetStatus().ToString()
            };
            
            if (_controller.GetStatus() == RobotStatus.Idle) {
                ReactToCmdVel(rosLinearSpeed, rosRotationSpeed);
            }

            _tick++;
        }

        void ReactToCmdVel(float speed, float rotSpeed) {
            // We prioritise rotation over movement
            if (Math.Abs(rotSpeed) > 0.01) {
                var degrees = 10 * rotSpeed;
                Debug.Log($"Turning {degrees} degrees");
                _controller.Rotate(degrees);
            } else if (rosLinearSpeed > 0) {
                var distanceInMeters = Mathf.Min(0.4f * speed, 0.2f);
                Debug.Log($"Moving forward in meters {distanceInMeters}");
                _controller.Move(distanceInMeters);
            }
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

        void ReceiveRosCmd(TwistMsg cmdVel) {
            rosLinearSpeed = (float)cmdVel.linear.x;
            rosRotationSpeed = (float)cmdVel.angular.z;
        }


        private void PostGarbageToScan() {
            var timestamp = new TimeStamp(Clock.time);

           var ranges = new List<float>();

           for (int i = 0; i < 180; i++) {
               ranges.Add((float) i);
           }
            
            var msg = new LaserScanMsg
            {
                header = new HeaderMsg
                {
                    frame_id = "base_scan",
                    stamp = new TimeMsg
                    {
                        sec = timestamp.Seconds,
                        nanosec = timestamp.NanoSeconds,
                    }
                },
                range_min = 0.12f,
                range_max = 100f,
                angle_min = 0,
                angle_max = Mathf.PI * 2,
                angle_increment = Mathf.Deg2Rad * 2f,
                time_increment = 0f,
                scan_time = 0.1f,
                intensities = new float[ranges.Count],
                ranges = ranges.ToArray(),
            };
        
            _ros.Publish(_rayTraceTopic, msg);
        }

        public string GetDebugInfo() {
            return "";
        }
        
        public void SetController(Robot2DController controller) {
            this._controller = controller;
            
            Debug.Log("Inside Set Controller");
            
            _ros = ROSConnection.GetOrCreateInstance();
            // Register publisher on normal topic
            // _ros.RegisterPublisher<RobotStateMsg>(_stateTopicName);
            
            // Example of registering service with callback function
            // _ros.ImplementService<BroadcastRequest, BroadcastResponse>(_broadcastServiceTopicName, BroadcastMessage);
            
            // _ros.RegisterPublisher<LaserScanMsg>(_rayTraceTopic);
            _ros.RegisterPublisher(_rayTraceTopic, LaserScanMsg.k_RosMessageName);
			_ros.Subscribe<TwistMsg>(_cmdVelocityTopic, ReceiveRosCmd);
        }
        
        public object SaveState() {
            return null;
        }

        public void RestoreState(object stateInfo) {
            return;
        }
    }
}