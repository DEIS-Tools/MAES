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
using Maes.YamlConfig;

namespace Maes {

    // This class contains all settings related to an instance of an simulation
    public static class GlobalSettings {
        private static readonly string ConfigFileName;

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

        public static readonly bool IsRosMode = false;

        static GlobalSettings() {
            // Maes only loads config from yaml file when in Ros Mode
            if (!IsRosMode) {
                return;
            }

            var config = MaesYamlConfigLoader.LoadConfig();

            // Populating static GlobalSettings class.
            LogicTickDeltaMillis = config.GlobalSettings.LogicTicksDeltaMillis;
            PhysicsTickDeltaMillis = config.GlobalSettings.PhysicsTicksPerLogicUpdate;
            DrawCommunication = config.GlobalSettings.DrawCommunication;
            ShowEnvironmentTags = config.GlobalSettings.ShowEnvironmentTags;
            ShouldWriteCSVResults = config.GlobalSettings.ShouldWriteCsvResults;
            if (config.GlobalSettings.StatisticsResultPath.Length > 0 && config.GlobalSettings.StatisticsResultPath.ToLower() != "default") {
                var dir = new DirectoryInfo(config.GlobalSettings.StatisticsResultPath);
                if (dir.Exists) {
                    StatisticsOutPutPath = config.GlobalSettings.StatisticsResultPath;
                }
                else {
                    Debug.Log($"Directory {config.GlobalSettings.StatisticsResultPath} not found. Defaulting to 'Desktop/MaesStatistics");
                }
            }
            TicksPerStatsSnapShot = config.GlobalSettings.TicksPerStatsSnapshot;
            PopulateAdjacencyAndComGroupsEveryTick = config.GlobalSettings.PopulateAdjacencyAndCommGroupsEveryTick;
            TicksBeforeExplorationHeatMapCold = config.GlobalSettings.TicksBeforeExplorationHeatmapCold;
            TicksBeforeCoverageHeatMapCold = config.GlobalSettings.TicksBeforeCoverageHeatmapCold;
        }
    }
}