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

using System;
using UnityEngine;
using RosMessageTypes.BuiltinInterfaces;

namespace Unity.Robotics.Core
{
    internal readonly struct TimeStamp
    {
        public const double k_NanosecondsInSecond = 1e9f;

        // TODO: specify base time this stamp is measured against (Sim 0, time since application start, etc.)
        public readonly int Seconds;
        public readonly uint NanoSeconds;

        // (From Unity Time.time)
        public TimeStamp(double timeInSeconds)
        {
            var sec = Math.Floor(timeInSeconds);
            var nsec = (timeInSeconds - sec) * k_NanosecondsInSecond;
            // TODO: Check for negatives to ensure safe cast
            Seconds = (int)sec;
            NanoSeconds = (uint)nsec;
        }

        // (From a ROS2 Time message)
        TimeStamp(int sec, uint nsec)
        {
            Seconds = sec;
            NanoSeconds = nsec;
        }

        // NOTE: We could define these operators in a transport-specific extension package
        public static implicit operator TimeMsg(TimeStamp stamp)
        {
            return new TimeMsg(stamp.Seconds, stamp.NanoSeconds);
        }

        public static implicit operator TimeStamp(TimeMsg stamp)
        {
            return new TimeStamp(stamp.sec, stamp.nanosec);
        }
    }
}
