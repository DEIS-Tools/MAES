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

namespace Unity.Robotics.Core
{
    public static class Clock
    {
        // Since UnityScaled is the default Unity Time mode, we'll use that for this project
        // None of the other time modes are fully validated and guaranteed to be without issues
        public enum ClockMode
        {
            // Time delta scaled by simulation speed to the start of the current frame, since the beginning of the game
            UnityScaled,
            // Real time delta to the exact moment this function is called, since beginning of the game 
            // UnityUnscaled,
            // Real time delta since the Unix Epoch (1/1/1970 @ midnight)
            // UnixEpoch,
            // Examples of other potentially useful clock modes...
            // UnixEpochUtc,
            // DateTimeNow,
            // DateTimeNowUtc,
            // ExternalClock
        }

        public const double k_NanoSecondsInSeconds = 1e9;

        static readonly DateTime k_UnixEpoch = new DateTime(1970, 1, 1, 0, 0, 0, 0);
        // Time the application started, relative to Unix Epoch
        static readonly double k_StartTimeEpochSeconds = SecondsSinceUnixEpoch - Time.realtimeSinceStartupAsDouble;
        
        static double SecondsSinceUnixEpoch => (DateTime.Now - k_UnixEpoch).TotalSeconds;
        static double UnityUnscaledTimeSinceFrameStart => 
            Time.realtimeSinceStartupAsDouble - Time.unscaledTimeAsDouble;

        public static double TimeSinceFrameStart => Now - FrameStartTimeInSeconds;

        public static double FrameStartTimeInSeconds
        {
            get
            {
                return Mode switch
                {
                    // This might be an approximation... needs testing.
                    ClockMode.UnityScaled => Time.timeAsDouble,
                    // ClockMode.UnityUnscaled => Time.unscaledTimeAsDouble,
                    // ClockMode.UnixEpoch => k_StartTimeEpochSeconds + UnityUnscaledTimeSinceFrameStart,
                    _ => throw new NotImplementedException()
                };
            }
        }

        public static double NowTimeInSeconds
        {
            get
            {
                return Mode switch
                {
                    ClockMode.UnityScaled => Time.timeAsDouble + UnityUnscaledTimeSinceFrameStart * Time.timeScale,
                    // ClockMode.UnityUnscaled => Time.realtimeSinceStartupAsDouble,
                    // ClockMode.UnixEpoch => SecondsSinceUnixEpoch,
                    _ => throw new NotImplementedException()
                };
            }
        }
        
        // NOTE: Precision loss vs. other time measurements due to no deltaTimeAsDouble interface
        public static float DeltaTimeInSeconds
        {
            get
            {
                return Mode switch
                {
                    ClockMode.UnityScaled => Time.deltaTime,
                    _ => Time.unscaledDeltaTime,
                };
            }
        }

        public static ClockMode Mode = ClockMode.UnityScaled;

        // Simple interfaces for supporting commonly used vocabulary
        public static double Now => NowTimeInSeconds;
        public static double time => FrameStartTimeInSeconds;
        public static float deltaTime => DeltaTimeInSeconds;

        // WARNING: These functions could potentially mess up threaded access to this clock class.
        //          Would need to include some mutex locking to keep these calls thread-safe
        public static double GetFrameTime(ClockMode temporaryMode)
        {
            var originalMode = Mode;
            Mode = temporaryMode;
            var t = FrameStartTimeInSeconds;
            Mode = originalMode;
            return t;
        }

        public static double GetNowTime(ClockMode temporaryMode)
        {
            var originalMode = Mode;
            Mode = temporaryMode;
            var t = NowTimeInSeconds;
            Mode = originalMode;
            return t;
        }
    }
}