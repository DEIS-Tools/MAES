using System;
using System.Diagnostics.CodeAnalysis;
using System.IO;
using System.Linq;
using UnityEngine;
using YamlDotNet.Core;
using YamlDotNet.Serialization;
using YamlDotNet.Serialization.NamingConventions;

namespace Maes.YamlConfig {
    public static class MaesYamlConfigLoader {

        private static MaesConfigType PreloadedConfig = null;
        
        public static MaesConfigType LoadConfig() {
            if (PreloadedConfig != null) return PreloadedConfig;
            
            string ConfigFileName;
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
                return null;
            }

            var stream = new StreamReader(ConfigFileName);
            Debug.Log("Before deserializer");
            var deserializer = new DeserializerBuilder()
                .WithNamingConvention(UnderscoredNamingConvention.Instance)
                .Build(); 
            
            MaesConfigType config;
            try {
                config = deserializer.Deserialize<MaesConfigType>(stream);
            }
            catch (YamlException e) {
                Debug.LogException(e);
                return null;
            }
            catch (Exception e) {
                Debug.Log($"Caught exception: {e.Message}");
                return null;
            }

            PreloadedConfig = config; // Cache for later use
            return config;
        }
        
            
            
        [SuppressMessage("ReSharper", "MemberHidesStaticFromOuterClass")]
        public class MaesConfigType {
            public int[] RandomSeeds { get; set; }
            public int NumberOfRobots { get; set; }
            public GlobalSettingsType GlobalSettings { get; set; }
            public RobotConstraintsType RobotConstraints { get; set; }
            public EndCriteriaType EndCriteria { get; set; }
            public RobotSpawnConfigType RobotSpawnConfig { get; set; }
            public MapType Map { get; set; }

            public override string ToString() {
                return $"{nameof(RandomSeeds)}: {RandomSeeds}, {nameof(NumberOfRobots)}: {NumberOfRobots}, {nameof(GlobalSettings)}: {GlobalSettings}, {nameof(RobotConstraints)}: {RobotConstraints}, {nameof(EndCriteria)}: {EndCriteria}, {nameof(RobotSpawnConfig)}: {RobotSpawnConfig}, {nameof(Map)}: {Map}";
            }
        }    
        
        /// <summary>
        /// Only used for parsing YML-values into, as YamlDotNet doesn't support parsing into static members directly.
        /// Remember to make changes here as well, when you make changes to <see cref="GlobalSettings"/>.
        /// </summary>
        [SuppressMessage("ReSharper", "MemberHidesStaticFromOuterClass")]
        public class GlobalSettingsType {
            // Times per second that robot logic is updated
            public int LogicTicksDeltaMillis { get; set; } = 100;

            // Amount of physics steps to calculate between each robot logic tick
            // Physics tick rate = LogicTickDelta / PhysicsTicksPerLogicUpdate
            public int PhysicsTicksPerLogicUpdate { get; set; } = 10;

            // Debug visualizer
            public bool DrawCommunication { get; set; } = true;
            public bool ShowEnvironmentTags { get; set; } = true;

            // Statistics
            public bool ShouldWriteCsvResults { get; set; } = false;
            
            public string StatisticsResultPath { get; set; } = "Default";
            public int TicksPerStatsSnapshot { get; set; } = 10;
            public bool PopulateAdjacencyAndCommGroupsEveryTick { get; set; } = false;
            public int TicksBeforeExplorationHeatmapCold { get; set; } = 2400; // 10*60*4
            public int TicksBeforeCoverageHeatmapCold { get; set; } = 2400;    // 10*60*4

            public override string ToString() {
                return $"{nameof(LogicTicksDeltaMillis)}: {LogicTicksDeltaMillis}, {nameof(PhysicsTicksPerLogicUpdate)}: {PhysicsTicksPerLogicUpdate}, {nameof(DrawCommunication)}: {DrawCommunication}, {nameof(ShowEnvironmentTags)}: {ShowEnvironmentTags}, {nameof(ShouldWriteCsvResults)}: {ShouldWriteCsvResults}, {nameof(StatisticsResultPath)}: {StatisticsResultPath}, {nameof(TicksPerStatsSnapshot)}: {TicksPerStatsSnapshot}, {nameof(PopulateAdjacencyAndCommGroupsEveryTick)}: {PopulateAdjacencyAndCommGroupsEveryTick}, {nameof(TicksBeforeExplorationHeatmapCold)}: {TicksBeforeExplorationHeatmapCold}, {nameof(TicksBeforeCoverageHeatmapCold)}: {TicksBeforeCoverageHeatmapCold}";
            }
        }
            
            
        [SuppressMessage("ReSharper", "MemberHidesStaticFromOuterClass")]
        public class RobotConstraintsType {
            public float BroadcastRange { get; set; }
            public bool BroadcastBlockedByWalls { get; set; }
            public float SenseNearbyAgentsRange { get; set; }
            public bool SenseNearbyAgentsBlockedByWalls { get; set; }
            public bool AutomaticallyUpdateSlam { get; set; }
            public int SlamUpdateIntervalInTicks { get; set; }
            public int SlamSyncIntervalInTicks { get; set; }
            public float SlamPositionInaccuracy { get; set; }
            public bool DistributeSlam { get; set; }
            public float EnvironmentTagReadRange { get; set; }
            public float SlamRaytraceRange { get; set; }
            public float RelativeMoveSpeed { get; set; }
            public float AgentRelativeSize { get; set; }

            public override string ToString() {
                return $"{nameof(BroadcastRange)}: {BroadcastRange}, {nameof(BroadcastBlockedByWalls)}: {BroadcastBlockedByWalls}, {nameof(SenseNearbyAgentsRange)}: {SenseNearbyAgentsRange}, {nameof(SenseNearbyAgentsBlockedByWalls)}: {SenseNearbyAgentsBlockedByWalls}, {nameof(AutomaticallyUpdateSlam)}: {AutomaticallyUpdateSlam}, {nameof(SlamUpdateIntervalInTicks)}: {SlamUpdateIntervalInTicks}, {nameof(SlamSyncIntervalInTicks)}: {SlamSyncIntervalInTicks}, {nameof(SlamPositionInaccuracy)}: {SlamPositionInaccuracy}, {nameof(DistributeSlam)}: {DistributeSlam}, {nameof(EnvironmentTagReadRange)}: {EnvironmentTagReadRange}, {nameof(SlamRaytraceRange)}: {SlamRaytraceRange}, {nameof(RelativeMoveSpeed)}: {RelativeMoveSpeed}, {nameof(AgentRelativeSize)}: {AgentRelativeSize}";
            }
        }

        public class EndCriteriaType {
            public float? CoveragePercent { get; set; } = null;
            public float? ExplorationPercent { get; set; } = null;

            public override string ToString() {
                return $"{nameof(CoveragePercent)}: {CoveragePercent}, {nameof(ExplorationPercent)}: {ExplorationPercent}";
            }
        }

        public class SpawnTogetherType {
            public int? SuggestedStartingPointX { get; set; } = null;
            public int? SuggestedStartingPointY { get; set; } = null;

            public bool HasSuggestedStartingPoint => this. SuggestedStartingPointX != null && this.SuggestedStartingPointY != null;

            public override string ToString() {
                return $"{nameof(SuggestedStartingPointX)}: {SuggestedStartingPointX}, {nameof(SuggestedStartingPointY)}: {SuggestedStartingPointY}, {nameof(HasSuggestedStartingPoint)}: {HasSuggestedStartingPoint}";
            }
        }

        public class RobotSpawnConfigType {
            public bool? BiggestRoom { get; set; } = null;
            public SpawnTogetherType SpawnTogether { get; set; } = null;
            public bool? SpawnAtHallwayEnds { get; set; } = null;

            public override string ToString() {
                return $"{nameof(BiggestRoom)}: {BiggestRoom}, {nameof(SpawnTogether)}: {SpawnTogether}, {nameof(SpawnAtHallwayEnds)}: {SpawnAtHallwayEnds}";
            }
        }

        public class CaveConfigType {
            public int SmoothingRuns { get; set; } = 4;
            public int ConnectionPassageWidth { get; set; } = 4;
            public int RandomFillPercent { get; set; } = 40;
            public int WallThresholdSize { get; set; } = 10;
            public int RoomThresholdSize { get; set; } = 10;

            public override string ToString() {
                return $"{nameof(SmoothingRuns)}: {SmoothingRuns}, {nameof(ConnectionPassageWidth)}: {ConnectionPassageWidth}, {nameof(RandomFillPercent)}: {RandomFillPercent}, {nameof(WallThresholdSize)}: {WallThresholdSize}, {nameof(RoomThresholdSize)}: {RoomThresholdSize}";
            }
        }

        public class BuildingConfigType {
            public int MaxHallInPercent { get; set; } = 20;
            public int MinRoomSideLength { get; set; } = 6;
            public int DoorWidth { get; set; } = 2;
            public int DoorPadding { get; set; } = 2;
            public int RoomSplitChance { get; set; } = 85;

            public override string ToString() {
                return $"{nameof(MaxHallInPercent)}: {MaxHallInPercent}, {nameof(MinRoomSideLength)}: {MinRoomSideLength}, {nameof(DoorWidth)}: {DoorWidth}, {nameof(DoorPadding)}: {DoorPadding}, {nameof(RoomSplitChance)}: {RoomSplitChance}";
            }
        }

        public class MapType {
            public float WallHeight { get; set; }
            public int WidthInTiles { get; set; }
            public int HeightInTiles { get; set; }
            public int BorderSize { get; set; } = 1;
            public BuildingConfigType BuildingConfig { get; set; } = null;
            public CaveConfigType CaveConfig { get; set; } = null;

            public override string ToString() {
                return $"{nameof(WallHeight)}: {WallHeight}, {nameof(WidthInTiles)}: {WidthInTiles}, {nameof(HeightInTiles)}: {HeightInTiles}, {nameof(BorderSize)}: {BorderSize}, {nameof(BuildingConfig)}: {BuildingConfig}, {nameof(CaveConfig)}: {CaveConfig}";
            }
        }
        
        
    }
}