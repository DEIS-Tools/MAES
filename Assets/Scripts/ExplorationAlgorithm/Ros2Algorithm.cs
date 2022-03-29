using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading.Tasks;
using Maes.Robot;
using Maes.Robot.Task;
using RosMessageTypes.BuiltinInterfaces;
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

        private string _rayTraceTopic = "scan";
        private int _tick = 0;

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
            // _ros.Publish(_stateTopicName, state);

            if(_tick % 10 == 0 && _tick > 200)
                _controller.Move(1);
                // PostGarbageToScan();
            // PublishGarbageClock();
            
            _tick++;
        }

        private void PublishGarbageClock() {
            var time = DateTime.Now;
            var clockMsg = new TimeMsg
            {
                sec = (int)time.Second,
                nanosec = (uint)(time.Millisecond * 1000000)
            };
            _ros.Publish("clock", clockMsg);
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
            // _ros.RegisterPublisher<ClockMsg>("clock");
            // _ros.RegisterPublisher<ClockMsg>("clock4");
            // Example of action
            // _ros.RegisterPublisher<MoveFeedback>(_rosRobotActionTopicName);
            // MoveActionFeedback.Register();
            // MoveActionGoal.Register();
            // MoveActionResult.Register();
            // _ros.RegisterPublisher(_rosRobotActionTopicName, MoveActionFeedback.k_RosMessageName);
            // _ros.Subscribe<MoveActionGoal>(_rosRobotActionTopicName, ExecuteMoveAction);
        }

        private void ExecuteMoveAction(MoveActionGoal goal) {
            _moveDistance = goal.goal.distance;
            Debug.Log($"Received move goal {goal.ToString()}");
        }

        private Task<BroadcastResponse> BroadcastMessage(BroadcastRequest request) {
            _controller.Broadcast($"{request.msg}");
            var response = new BroadcastResponse {
                success_status = "Success"
            };
            return Task.FromResult(response);
        }

        public object SaveState() {
            return null;
        }

        public void RestoreState(object stateInfo) {
            return;
        }
    }
}