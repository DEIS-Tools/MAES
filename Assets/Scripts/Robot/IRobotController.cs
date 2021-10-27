using System;
using System.Collections.Generic;

namespace Dora.Robot
{
    public interface IRobotController
    {

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
        void StartRotating(bool counterClockwise=false);

        /* Stops performing the current task (rotating or moving) */
        void StopCurrentTask();
        
        // Broadcasts the given data to all robots within range (range determined by simulation configuration)
        // The data will be available to nearby robots in the next logic update
        void Broadcast(object data);
        
        // Receives broadcast data sent by nearby robots the previous logic tick
        List<object> ReceiveBroadcast();

        // TODO:
        // Returns the current SLAM map 
        // SlamMap UpdateSlamMap()
        
        // Synchronizes the current the current SLAM map with nearby robots.
        // Synchronization will be completed in the next logic update
        // void SynchronizeSlamMap()
        
        // + Sense nearby robots and walls
    }
    
}
