// Copyright 2022 MAES
// 
// This file is part of MAES
// 
// MAES is free software: you can redistribute it and/or modify it under
// the terms of the GNU General Public License as published by the
// Free Software Foundation, either version 3 of the License, or (at your option)
// any later version.
// 
// MAES is distributed in the hope that it will be useful, but WITHOUT
// ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
// or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
// Public License for more details.
// 
// You should have received a copy of the GNU General Public License along
// with MAES. If not, see http://www.gnu.org/licenses/.
// 
// Contributors: Malte Z. Andreasen, Philip I. Holler and Magnus K. Jensen
// 
// Original repository: https://github.com/MalteZA/MAES

#nullable enable
using System;
using JetBrains.Annotations;

namespace Maes.Robot
{
    public class RobotConstraints
    {
        public delegate bool SignalTransmissionSuccessCalculator(float totalDistance, float distanceThroughWalls);

        public SignalTransmissionSuccessCalculator IsTransmissionSuccessful { get; }

        public float SenseNearbyAgentsRange { get; }
        public bool SenseNearbyAgentsBlockedByWalls { get; }
        public bool MaterialCommunication { get; }
        // Given in MHz
        public int Frequency { get; }
        // In dBm
        public int TransmitPower { get; }
        public int ReceiverSensitivity { get; }

        // SLAM
        public bool AutomaticallyUpdateSlam { get; }
        public int SlamUpdateIntervalInTicks { get; }
        public int SlamSynchronizeIntervalInTicks { get; }
        public float SlamPositionInaccuracy { get; }
        public bool DistributeSlam { get; }


        // *** Ray cast range and count affects both exploration progress and the slam map ***

        // The amount of traces shot out, higher trace count will result in a more complete map
        // The smaller the ray cast range the lower amount of traces is needed
        // If ray cast count is left unspecified, a default amount will be calculated from the ray cast rang
        public int? SlamRayTraceCount { get; }

        // SLAM ray trace range
        public float SlamRayTraceRange { get; }

        // Environment tagging
        public float EnvironmentTagReadRange { get; }

        // Movement
        // 1.0f is default. A bigger map with bigger doors would make the robot "feel" slower. It is thus not 
        // a speed value in e.g. km/h .
        public float RelativeMoveSpeed { get; }
        public float AgentRelativeSize { get; }


        public RobotConstraints(
            float senseNearbyAgentsRange = 20f,
            bool senseNearbyAgentsBlockedByWalls = true,
            bool automaticallyUpdateSlam = true,
            int slamUpdateIntervalInTicks = 10,
            int slamSynchronizeIntervalInTicks = 10,
            float slamPositionInaccuracy = 0.2f,
            bool distributeSlam = false,
            float environmentTagReadRange = 2f,
            float slamRayTraceRange = 20f,
            float relativeMoveSpeed = 1f,
            float agentRelativeSize = 0.6f,
            SignalTransmissionSuccessCalculator? calculateSignalTransmissionProbability = null,
            int? slamRayTraceCount = null,
            bool materialCommunication = true,
            int frequency = 2400,
            int transmitPower = 15,
            int receiverSensitivity = 5)
        {

            SenseNearbyAgentsRange = senseNearbyAgentsRange;
            SenseNearbyAgentsBlockedByWalls = senseNearbyAgentsBlockedByWalls;

            // Materials
            MaterialCommunication = materialCommunication;
            Frequency = frequency;
            TransmitPower = transmitPower;
            ReceiverSensitivity = receiverSensitivity;

            // SLAM
            AutomaticallyUpdateSlam = automaticallyUpdateSlam;
            SlamUpdateIntervalInTicks = slamUpdateIntervalInTicks;
            SlamSynchronizeIntervalInTicks = slamSynchronizeIntervalInTicks;
            SlamPositionInaccuracy = slamPositionInaccuracy;
            SlamRayTraceRange = slamRayTraceRange;
            SlamRayTraceCount = slamRayTraceCount;
            DistributeSlam = distributeSlam;

            EnvironmentTagReadRange = environmentTagReadRange;
            RelativeMoveSpeed = relativeMoveSpeed;
            AgentRelativeSize = agentRelativeSize;

            // Communication/Broadcasting
            IsTransmissionSuccessful = calculateSignalTransmissionProbability ?? ((distance, _) => true);

        }
    }
}