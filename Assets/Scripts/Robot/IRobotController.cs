using System;
using System.Collections.Generic;
using Maes.Map;
using Maes.Robot.Task;

namespace Maes.Robot {
    public interface IRobotController {
        
        /// <returns> The unique integer id of this robot </returns>
        int GetRobotID();
        
        /// <summary>
        /// Gives the current state of the robot which can have value of {Idle, Moving, Stopping}.
        /// 'Idle' indicates that the robot is not performing a task and is not moving
        /// 'Moving' indicates that the robot is performing a movement based task
        /// 'Stopping' indicates that the robot has stopped its current task, but has not yet fully stopped moving
        /// </summary>
        /// <returns> the current RobotStatus, which can be either Idle, Moving or Stopping</returns>
        RobotStatus GetStatus();

        /// <summary>
        /// Used for sensing new collisions. A collision is only considered new at the moment of first touch.
        /// If the robot continues to touch the collided object, this method will not return true.
        /// For information about whether or not the robot is currently touching another object use the
        /// IsCurrentlyColliding() method
        /// </summary>
        /// <returns> True if the robot has encountered a new collision since the last logic tick and false if not </returns>
        bool HasCollidedSinceLastLogicTick();
        
        /// <returns> True if the robot is currently touching another object (a wall or another robot). </returns>
        bool IsCurrentlyColliding();

        /// <summary>
        /// This method instructs the robot to move <paramref name="distanceInMeters"/>ahead.
        /// The distance is approximated by the robot, and accuracy will depend on the implementation.
        /// The instruction is cancelled if the robot encounters a collision.
        /// This instruction can be manually cancelled by calling <see cref="StopCurrentTask"/>
        /// </summary>
        /// <param name="distanceInMeters"> The distance that the robt should attempt to move ahead</param>
        /// <param name="reverse"> Move backwards if true and forwards if false</param>
        void Move(float distanceInMeters, bool reverse = false);

        /// <summary>
        /// This method instructs the robot to start moving ahead.
        /// Movement will continue until the instruction is cancelled, either manually or when colliding.
        /// This instruction can be manually cancelled by calling <see cref="StopCurrentTask"/>
        /// </summary>
        /// <param name="reverse"> Move backwards if true and forwards if false</param>
        void StartMoving(bool reverse = false);

        /// <summary>
        /// Instructs the robot to rotate <paramref name="degrees"/>.
        /// Rotation will continue until completed or until cancelled, either manually or by a collision.
        /// This instruction can be manually cancelled by calling <see cref="StopCurrentTask"/>
        /// </summary>
        /// <param name="degrees"> The amount of degrees to rotate. Must be in range [-180, 180]</param>
        void Rotate(float degrees);

        /// <summary>
        /// This method instructs the robot to start rotating clockwise in place.
        /// Rotation will continue until the instruction is cancelled, either manually or by a collision.
        /// This instruction can be manually cancelled by calling <see cref="StopCurrentTask"/>
        /// </summary>
        /// <param name="counterClockwise"> Rotate counterclockwise if true and clockwise if false </param>
        void StartRotating(bool counterClockwise = false);

        /// <summary>
        /// Stops the currently active task. Does nothing if the robot has no task.
        /// </summary>
        void StopCurrentTask();

        /// <summary>
        /// Broadcasts the <paramref name="data"/> to all robots that are within range (range is determined by the
        /// simulation's robot configuration) and in line of sight (if configured in the simulation's
        /// <see cref="RobotConstraints"/>).
        /// The data will be available to nearby robots in the next logic update.
        /// </summary>
        /// <param name="data">The message of any type that will be delivered to nearby robots next logic tick</param>
        void Broadcast(object data);

        /// <summary>
        /// Receives all broadcast data sent by nearby robots the previous logic tick
        /// </summary>
        /// <returns> a list of message objects of any type </returns>
        List<object> ReceiveBroadcast();

        /// <summary>
        /// Deposits the given tag into the environment at the current position of the robot.
        /// Once placed a tag cannot be removed.
        /// </summary>
        /// <param name="tag">The tag of type ITag that will be deposited</param>
        void DepositTag(EnvironmentTaggingMap.ITag tag);
        
        
        /// <summary>
        /// Reads tags that are within tag reading range (determined by the RobotConfiguration of the simulation)
        /// </summary>
        /// <returns> a list of all the tags within tag reading range of the robot</returns>
        List<RelativeObject<EnvironmentTaggingMap.ITag>> ReadNearbyTags();
        
        public readonly struct DetectedWall {
            public readonly float distance;
            public readonly float relativeAngle;

            public DetectedWall(float distance, float relativeAngle) {
                this.distance = distance;
                this.relativeAngle = relativeAngle;
            }
        }

        /// <summary>
        /// Attempts to detect the nearest wall in the given angle (<paramref name="globalAngle"/>)
        /// A trace is fired from each wheel, to detect walls that
        /// Walls will only be detected if within lidar range configured in <see cref="RobotConstraints"/>
        /// </summary>
        /// <param name="globalAngle">The global angle, relative to the x-axis in which the ray traces are fired</param>
        /// <returns>a DetectedWall object if a wall is found, otherwise null</returns>
        public DetectedWall? DetectWall(float globalAngle);
        
        /// <returns>the global orientation of the robot measured in degrees from the x axis (counter clockwise)
        /// in the range [0-360]</returns>
        public float GetGlobalAngle();

        /// <summary>
        /// Returns debugging info specific to the controller implementation.
        /// </summary>
        String GetDebugInfo();

        /// <summary>
        /// Senses robots within range (configured in the simulation's <see cref="RobotConstraints"/>).
        /// </summary>
        /// <returns>a list of relative positions (containing integer ids of sensed robots) </returns>
        List<CommunicationManager.SensedObject<int>> SenseNearbyRobots();

        /// <summary>
        /// This yields the slam map that is automatically generated by this robot (if enabled in this
        /// simulation's <see cref="RobotConstraints"/>)
        /// </summary>
        /// <returns>a reference to the robots <see cref="SlamMap"/></returns>
        ISlamAlgorithm GetSlamMap();
    }
}