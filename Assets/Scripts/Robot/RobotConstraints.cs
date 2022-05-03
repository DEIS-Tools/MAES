#nullable enable
using System;
using JetBrains.Annotations;
using TMPro;

namespace Maes.Robot {
    public readonly struct RobotConstraints {
        public readonly float SenseNearbyRobotRange;
        public readonly bool SenseNearbyRobotBlockedByWalls;
        
        public delegate float SignalTransmissionProbability(float totalDistance, float distanceThroughWalls);

        public readonly SignalTransmissionProbability calculateSignalTransmissionProbability;
        // The cutoff (in units provided by the SignalTransmissionProbability function) at which signals will no
        // longer be transmitted
        public readonly float MinimumSignalTransmissionProbability;

        public readonly float SenseNearbyAgentsRange;
        public readonly bool SenseNearbyAgentsBlockedByWalls;

        // SLAM
        public readonly bool AutomaticallyUpdateSlam;
        public readonly int SlamUpdateIntervalInTicks;
        public readonly int SlamSynchronizeIntervalInTicks;
        public readonly float SlamPositionInaccuracy;
        public readonly bool DistributeSlam;


        // *** Ray cast range and count affects both exploration progress and the slam map ***

        // The amount of traces shot out, higher trace count will result in a more complete map
        // The smaller the ray cast range the lower amount of traces is needed
        // If ray cast count is left unspecified, a default amount will be calculated from the ray cast rang
        public readonly int? SlamRayTraceCount;

        // SLAM ray trace range
        public readonly float SlamRayTraceRange;

        // Environment tagging
        public readonly float EnvironmentTagReadRange;

        // Movement
        // 1.0f is default. A bigger map with bigger doors would make the robot "feel" slower. It is thus not 
        // a speed value in e.g. km/h .
        public readonly float RelativeMoveSpeed;


        public readonly float RobotRelativeSize;

        // TODO: Add robot size to constraints class
        
        public readonly float AgentRelativeSize;


        public RobotConstraints(float broadcastRange, bool broadcastBlockedByWalls, float senseNearbyAgentsRange,
            bool senseNearbyAgentsBlockedByWalls, bool automaticallyUpdateSlam, int slamUpdateIntervalInTicks,
            int slamSynchronizeIntervalInTicks, float slamPositionInaccuracy, bool distributeSlam,
            float environmentTagReadRange, float slamRayTraceRange, float relativeMoveSpeed, float agentRelativeSize, 
            SignalTransmissionProbability? calculateSignalTransmissionProbability = null, 
            float minimumSignalTransmissionProbability = 0.9f, int? slamRayTraceCount = null) : this() {

            SenseNearbyAgentsRange = senseNearbyAgentsRange;
            SenseNearbyAgentsBlockedByWalls = senseNearbyAgentsBlockedByWalls;

            // SLAM
            AutomaticallyUpdateSlam = automaticallyUpdateSlam;
            SlamUpdateIntervalInTicks = slamUpdateIntervalInTicks;
            SlamSynchronizeIntervalInTicks = slamSynchronizeIntervalInTicks;
            SlamPositionInaccuracy = slamPositionInaccuracy;
            SlamRayTraceRange = slamRayTraceRange;
            SlamRayTraceCount = slamRayTraceCount;
            DistributeSlam = distributeSlam;

            // SLAM
            EnvironmentTagReadRange = environmentTagReadRange;
            RelativeMoveSpeed = relativeMoveSpeed;
            AgentRelativeSize = agentRelativeSize;
            
            // Communication/Broadcasting
            this.calculateSignalTransmissionProbability = calculateSignalTransmissionProbability ?? ((distance, _) => 100.0f);
            MinimumSignalTransmissionProbability = minimumSignalTransmissionProbability;
        }
    }
}