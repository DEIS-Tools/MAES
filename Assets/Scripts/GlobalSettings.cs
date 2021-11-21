using System;
using System.IO;

namespace Dora {
    // This class contains all settings related to an instance of an simulation
    public static class GlobalSettings {
        // Times per second that robot logic is updated
        public static readonly int LogicTickDeltaMillis = 100;

        // Amount of physics steps to calculate between each robot logic tick
        // Physics tick rate = LogicTickDelta / PhysicsTicksPerLogicUpdate
        public static readonly int PhysicsTicksPerLogicUpdate = 10;

        public static readonly int PhysicsTickDeltaMillis = LogicTickDeltaMillis / PhysicsTicksPerLogicUpdate;
        public static readonly float PhysicsTickDeltaSeconds = PhysicsTickDeltaMillis / 1000f;

        // Debug visualizer
        public static readonly bool DrawCommunication = true;
        public static readonly bool ShowEnvironmentTags = true;

        // Statistics
        public static readonly bool ShouldWriteCSVResults = true;
        public static readonly int TicksPerStatsSnapShot = 50;

        public static readonly string StatisticsOutPutPath =
            Environment.GetFolderPath(Environment.SpecialFolder.Desktop) 
            + Path.DirectorySeparatorChar + "MaesStatistics" + Path.DirectorySeparatorChar;
    }
}