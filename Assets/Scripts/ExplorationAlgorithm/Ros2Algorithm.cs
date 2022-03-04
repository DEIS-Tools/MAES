using System;
using Maes.Robot;
using Maes.Robot.Task;
using ROS2;
using UnityEngine;

namespace Maes.ExplorationAlgorithm {
    public class Ros2Algorithm : IExplorationAlgorithm {
        private Robot2DController _controller;
        private ROS2UnityComponent _ros2UnityComponent;
        private ISubscription<geometry_msgs.msg.Twist> twist_sub;
        private ROS2Node _ros2Node;
        private Movement? nextMove = null;


        private enum Movement {
            FORWARD, REVERSE, RIGHT, LEFT
        }

        public void UpdateLogic() {
            if (_controller.GetStatus() == RobotStatus.Idle) {
                switch (nextMove)
                {
                    case Movement.FORWARD:
                        _controller.Move(1);
                        nextMove = null;
                        break;
                    case Movement.REVERSE:
                        _controller.Move(1);
                        nextMove = null;
                        break;
                    case Movement.RIGHT:
                        _controller.Rotate(-90);
                        nextMove = null;
                        break;
                    case Movement.LEFT:
                        _controller.Rotate(90);
                        nextMove = null;
                        break;
                    case null:
                        break;
                    default:
                        throw new ArgumentOutOfRangeException();
                }
            }
        }

        public void SetUnityComponent(ROS2UnityComponent component) {
            this._ros2UnityComponent = component;
            this._ros2Node = _ros2UnityComponent.CreateNode($"Robot{_controller.GetRobotID()}");
            twist_sub = _ros2Node.CreateSubscription<geometry_msgs.msg.Twist>(
                "turtle1/cmd_vel", msg => QueueReaction(msg));
        }
        
        
        public string GetDebugInfo() {
            return "";
        }

        private void QueueReaction(geometry_msgs.msg.Twist twist) {
            if (twist.Linear.X > 0) {
                nextMove = Movement.FORWARD;
            }
            else if (twist.Linear.X > 0) {
                nextMove = Movement.REVERSE;
            }
            else if (twist.Angular.Z != 0) {
                nextMove = twist.Angular.Z > 0 ? Movement.LEFT : Movement.RIGHT;
            }
            
            
        }

        public void SetController(Robot2DController controller) {
            this._controller = controller;
        }

        public object SaveState() {
            return null;
        }

        public void RestoreState(object stateInfo) {
            return;
        }
    }
}