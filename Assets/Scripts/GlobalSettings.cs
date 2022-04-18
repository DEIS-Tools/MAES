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

            var config = MaesYamlConfigLoader.LoadConfig("");

            // Populating static GlobalSettings class.
            LogicTickDeltaMillis = config.global_settings.logic_ticks_delta_millis;
            PhysicsTickDeltaMillis = config.global_settings.physics_ticks_per_logic_update;
            DrawCommunication = config.global_settings.draw_communication;
            ShowEnvironmentTags = config.global_settings.show_environment_tags;
            ShouldWriteCSVResults = config.global_settings.should_write_csv_results;
            if (config.global_settings.statistics_result_path.Length > 0 && config.global_settings.statistics_result_path.ToLower() != "default") {
                var dir = new DirectoryInfo(config.global_settings.statistics_result_path);
                if (dir.Exists) {
                    StatisticsOutPutPath = config.global_settings.statistics_result_path;
                }
                else {
                    Debug.Log($"Directory {config.global_settings.statistics_result_path} not found. Defaulting to 'Desktop/MaesStatistics");
                }
            }
            TicksPerStatsSnapShot = config.global_settings.ticks_per_stats_snapshot;
            PopulateAdjacencyAndComGroupsEveryTick = config.global_settings.populate_adjacency_and_comm_groups_every_tick;
            TicksBeforeExplorationHeatMapCold = config.global_settings.ticks_before_exploration_heatmap_cold;
            TicksBeforeCoverageHeatMapCold = config.global_settings.ticks_before_coverage_heatmap_cold;

            // Ignore further changes to config file to avoid confusion.
            IgnoreConfigFiles = true;
        }
    }
}