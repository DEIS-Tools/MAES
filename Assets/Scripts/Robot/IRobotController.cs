using System;
using System.Collections.Generic;
using static Dora.MapGeneration.EnvironmentTaggingMap;

namespace Dora.Robot {
    public interface IRobotController {
        int GetRobotID();

        // Returns information about the robots current state {Idle, Moving, Stopping}
        // The robot can only accept instructions if it is in the 'Idle' state 
        RobotStatus GetStatus();

        // Returns true if the robot has encountered a new collision since the previous logic update
        bool HasCollided();

        // Instructs the robot to move forward until it has travelled **approximately** the given distance.
        void Move(float distanceInMeters, bool reverse = false);

        // Instructs the robot to start moving backwards. Continues until encountering a collision or until
        // stopped through a StopCurrentTask()
        void StartMoving(bool reverse = false);

        // Instruct the robot to keep rotating the robot until the robot has rotated **approximately**
        // the given amount of degrees 
        void Rotate(float degrees);

        // Instruct the robot to start rotating every until stopped through StopCurrentAction()
        void StartRotating(bool counterClockwise = false);

        /* Stops performing the current task (rotating or moving) */
        void StopCurrentTask();

        // Broadcasts the given data to all robots within range (range determined by simulation configuration)
        // The data will be available to nearby robots in the next logic update
        void Broadcast(object data);

        // Receives broadcast data sent by nearby robots the previous logic tick
        List<object> ReceiveBroadcast();

        // Deposits a tag into the environment at the current position of the robot
        void DepositTag(object data);
        
        // Returns a list of all environment tags that are within sensor range 
        List<EnvironmentTag> ReadNearbyTags();

        // Returns debugging information about the robot that will be shown when the robot is selected
        String GetDebugInfo();

        List<CommunicationManager.SensedObject<int>> SenseNearbyRobots();

        SlamAlgorithmInterface GetSlamMap();
    }
}