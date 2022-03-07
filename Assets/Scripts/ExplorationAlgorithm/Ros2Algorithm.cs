using System;
using Maes.Robot;
using Maes.Robot.Task;
using RosMessageTypes.MaesInterface;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;

namespace Maes.ExplorationAlgorithm {
    public class Ros2Algorithm : IExplorationAlgorithm {
        private Robot2DController _controller;
        private ROSConnection _ros;
        private string _topicName = "robot1/state_msg";
        private string _serviceName = "robot1/broadcast_srv";


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
            _ros.Publish(_topicName, state);
        }

        public string GetDebugInfo() {
            return "";
        }
        
        public void SetController(Robot2DController controller) {
            this._controller = controller;
            
            _ros = ROSConnection.GetOrCreateInstance();
            // Register publisher on normal topic
            _ros.RegisterPublisher<RobotStateMsg>(_topicName);
            // Example of registering service with callback function
            _ros.ImplementService<BroadcastRequest, BroadcastResponse>(_serviceName, BroadcastMessage);
        }

        private BroadcastResponse BroadcastMessage(BroadcastRequest request) {
            _controller.Broadcast($"{request.msg}");
            var response = new BroadcastResponse {
                success_status = "Success"
            };
            return response;
        }

        public object SaveState() {
            return null;
        }

        public void RestoreState(object stateInfo) {
            return;
        }
    }
}