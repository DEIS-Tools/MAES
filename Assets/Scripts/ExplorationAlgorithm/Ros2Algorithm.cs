using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading.Tasks;
using Maes.Robot;
using Maes.Robot.Task;
using RosMessageTypes.BuiltinInterfaces;
using RosMessageTypes.Geometry;
using RosMessageTypes.Sensor;
using RosMessageTypes.Std;
using Unity.Robotics.Core;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;

namespace Maes.ExplorationAlgorithm {
    public class Ros2Algorithm : IExplorationAlgorithm {
        private Robot2DController _controller;
        private ROSConnection _ros;

        private string _cmdVelocityTopic = "/cmd_vel";
        private string _rayTraceTopic = "/scan";
        private int _tick = 0;
        
        private float rosLinearSpeed = 0f;
        private float rosRotationSpeed = 0f;

        public void UpdateLogic() {
            ReactToCmdVel(rosLinearSpeed, rosRotationSpeed);
            _tick++;
        }
        
        void ReactToCmdVel(float speedCommandValue, float rotationCommandValue) {
            Debug.Log($"Command velocities: [{speedCommandValue}, {rotationCommandValue}]");
            var robotStatus = _controller.GetStatus();
            // We prioritise rotation over movement - However if desired rotation is very small (<0.1) we ignore it
            if (Math.Abs(rotationCommandValue) > 0.08) {
                if (robotStatus != RobotStatus.Idle && !_controller.IsRotatingIndefinitely()) {
                    // The robot is currently performing another task - Stop that task and continue
                    _controller.StopCurrentTask();
                } else {
                    var sign = rotationCommandValue > 0 ? -1 : 1;
                    var force = Mathf.Pow(Math.Abs(1.3f * rotationCommandValue), 2.0f) * 0.6f;
                    force = Mathf.Min(1f, force); // Ensure maximum force of 1.0 
                    force = sign * force; // Apply direction / sign +-
                    _controller.RotateAtRate(force);
                }
                // var degrees = 10 * rotationCommandValue;
                // Debug.Log($"Turning {degrees} degrees");
                // _controller.Rotate(degrees);
            } else if (rosLinearSpeed > 0) {
                if (robotStatus != RobotStatus.Idle && _controller.IsRotating()) {
                    // The robot must stop rotation task before starting the desired movement task
                    _controller.StopCurrentTask();
                } else if (robotStatus == RobotStatus.Idle) {
                    _controller.MoveAtRate(speedCommandValue);
                }
                // var distanceInMeters = Mathf.Min(0.4f * speedCommandValue, 0.2f);
                // Debug.Log($"Moving forward in meters {distanceInMeters}");
                // _controller.StartMoving(); //.Move(distanceInMeters);
            } else if (_controller.GetStatus() != RobotStatus.Idle){
                // If cmd_vel does not indicate any desired movement - then stop robot if currently moving 
                Debug.Log("Stopping movement!");
                _controller.StopCurrentTask();    
            }
        }

        void ReceiveRosCmd(TwistMsg cmdVel) {
            rosLinearSpeed = (float)cmdVel.linear.x;
            rosRotationSpeed = (float)cmdVel.angular.z;
            Debug.Log($"Robot {_controller.GetRobotID()}: Received cmdVel twist: {cmdVel.ToString()}");
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
            _ros = ROSConnection.GetOrCreateInstance();

            var cmdVelTopic = $"/robot{_controller.GetRobotID()}/cmd_vel";
            _ros.Subscribe<TwistMsg>(cmdVelTopic, ReceiveRosCmd);
        }
        
        public object SaveState() {
            return null;
        }

        public void RestoreState(object stateInfo) {
            return;
        }
    }
}