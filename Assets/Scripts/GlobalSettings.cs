using System;
using System.Diagnostics.CodeAnalysis;
using System.IO;
using System.Linq;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using UnityEngine;
using YamlDotNet.Core;
using YamlDotNet.Serialization;
using YamlDotNet.Serialization.NamingConventions;

namespace Maes {
    // This class contains all settings related to an instance of an simulation
    public static class GlobalSettings {
        private static readonly string ConfigFileName;
        private static readonly bool IgnoreConfigFiles = false;

        // Times per second that robot logic is updated
        public static readonly int LogicTickDeltaMillis = 100;

        // Amount of physics steps to calculate between each robot logic tick
        // Physics tick rate = LogicTickDelta / PhysicsTicksPerLogicUpdate
        public static readonly int PhysicsTicksPerLogicUpdate = 10;

        // Debug visualizer
        public static readonly bool DrawCommunication = true;
        public static readonly bool ShowEnvironmentTags = true;

        // Statistics
        public static readonly bool ShouldWriteCSVResults = false;
        public static readonly string StatisticsOutPutPath =
            Environment.GetFolderPath(Environment.SpecialFolder.Desktop)
            + Path.DirectorySeparatorChar + "MaesStatistics" + Path.DirectorySeparatorChar;
        public static readonly int TicksPerStatsSnapShot = 10;
        public static readonly bool PopulateAdjacencyAndComGroupsEveryTick = false;


        // The below constants depend on the above constants. Do not change this individually!
        public static readonly int PhysicsTickDeltaMillis = LogicTickDeltaMillis / PhysicsTicksPerLogicUpdate;
        public static readonly float PhysicsTickDeltaSeconds = PhysicsTickDeltaMillis / 1000f;

        public static readonly int TicksBeforeExplorationHeatMapCold = 10 * 60 * 4;
        public static readonly int TicksBeforeCoverageHeatMapCold = 10 * 60 * 4;

        static GlobalSettings() {
            if (IgnoreConfigFiles) {
                return;
            }
            
            try {
                var yFile = new DirectoryInfo(Directory.GetCurrentDirectory())
                    .GetFiles("*.y*ml")
                    .Where(f => f.Name.ToLower().Contains("config"))
                    .OrderByDescending(f => f.LastWriteTime)
                    .First();
                ConfigFileName = yFile.FullName;
                Debug.Log($"Found {yFile.Name} as config-file. ({yFile.FullName})");
            }
            catch (InvalidOperationException e) {
                Debug.Log("No valid config-file found. Defaulting to hard-coded settings from GlobalSettings.cs");
                ConfigFileName = null;
                return;
            }

            var stream = new StreamReader(ConfigFileName);
            var deserializer = new DeserializerBuilder()
                .WithNamingConvention(PascalCaseNamingConvention.Instance)
                .Build();

            GlobalSettingsType s;
            try {
                s = deserializer.Deserialize<GlobalSettingsType>(stream);
            }
            catch (YamlException e) {
                Debug.LogException(e);
                return;
            }

            // Populating static GlobalSettings class.
            LogicTickDeltaMillis = s.LogicTickDeltaMillis;
            PhysicsTickDeltaMillis = s.PhysicsTicksPerLogicUpdate;
            DrawCommunication = s.DrawCommunication;
            ShowEnvironmentTags = s.ShowEnvironmentTags;
            ShouldWriteCSVResults = s.ShouldWriteCSVResults;
            if (s.StatisticsOutPutPath.Length > 0 && s.StatisticsOutPutPath.ToLower() != "default") {
                var dir = new DirectoryInfo(s.StatisticsOutPutPath);
                if (dir.Exists) {
                    StatisticsOutPutPath = s.StatisticsOutPutPath;
                }
                else {
                    Debug.Log($"Directory {s.StatisticsOutPutPath} not found. Defaulting to 'Desktop/MaesStatistics");
                }
            }
            TicksPerStatsSnapShot = s.TicksPerStatsSnapShot;
            PopulateAdjacencyAndComGroupsEveryTick = s.PopulateAdjacencyAndComGroupsEveryTick;
            TicksBeforeExplorationHeatMapCold = s.TicksBeforeExplorationHeatMapCold;
            TicksBeforeCoverageHeatMapCold = s.TicksBeforeCoverageHeatMapCold;

            // Ignore further changes to config file to avoid confusion.
            IgnoreConfigFiles = true;
        }
        
        /// <summary>
        /// Only used for parsing YML-values into, as YamlDotNet doesn't support parsing into static members directly.
        /// Remember to make changes here as well, when you make changes to <see cref="GlobalSettings"/>.
        /// </summary>
        [SuppressMessage("ReSharper", "MemberHidesStaticFromOuterClass")]
        private class GlobalSettingsType {
            // Times per second that robot logic is updated
            public int LogicTickDeltaMillis { get; set; } = 100;

            // Amount of physics steps to calculate between each robot logic tick
            // Physics tick rate = LogicTickDelta / PhysicsTicksPerLogicUpdate
            public int PhysicsTicksPerLogicUpdate { get; set; } = 10;

            // Debug visualizer
            public bool DrawCommunication { get; set; } = true;
            public bool ShowEnvironmentTags { get; set; } = true;

            // Statistics
            public bool ShouldWriteCSVResults { get; set; } = false;
            public string StatisticsOutPutPath { get; set; } = "Default";
            public int TicksPerStatsSnapShot { get; set; } = 10;
            public bool PopulateAdjacencyAndComGroupsEveryTick { get; set; } = false;
            public int TicksBeforeExplorationHeatMapCold { get; set; } = 2400; // 10*60*4
            public int TicksBeforeCoverageHeatMapCold { get; set; } = 2400;    // 10*60*4
        }
    }
}