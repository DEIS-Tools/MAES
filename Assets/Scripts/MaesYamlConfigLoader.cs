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

        public static MaesConfigType LoadConfig(string Path) {
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
            
            Debug.Log($"Using config: \n{config}");
            return config;
        }
        
            
            
        [SuppressMessage("ReSharper", "MemberHidesStaticFromOuterClass")]
        public class MaesConfigType {
            public int[] random_seeds { get; set; }
            
            public int number_of_robots { get; set; }
            public GlobalSettingsType global_settings { get; set; }
            public RobotConstraintsType robot_constraints { get; set; }
            public EndCriteriaType end_criteria { get; set; }
            public RobotSpawnConfigType robot_spawn_config { get; set; }
            public MapType map { get; set; }

            public override string ToString() {
                return $"{nameof(random_seeds)}: {random_seeds}, {nameof(number_of_robots)}: {number_of_robots}, {nameof(global_settings)}: {global_settings}, {nameof(robot_constraints)}: {robot_constraints}, {nameof(end_criteria)}: {end_criteria}, {nameof(robot_spawn_config)}: {robot_spawn_config}, {nameof(map)}: {map}";
            }
        }    
        
        /// <summary>
        /// Only used for parsing YML-values into, as YamlDotNet doesn't support parsing into static members directly.
        /// Remember to make changes here as well, when you make changes to <see cref="GlobalSettings"/>.
        /// </summary>
        [SuppressMessage("ReSharper", "MemberHidesStaticFromOuterClass")]
        public class GlobalSettingsType {
            // Times per second that robot logic is updated
            public int logic_ticks_delta_millis { get; set; } = 100;

            // Amount of physics steps to calculate between each robot logic tick
            // Physics tick rate = LogicTickDelta / PhysicsTicksPerLogicUpdate
            public int physics_ticks_per_logic_update { get; set; } = 10;

            // Debug visualizer
            public bool draw_communication { get; set; } = true;
            public bool show_environment_tags { get; set; } = true;

            // Statistics
            public bool should_write_csv_results { get; set; } = false;
            
            public string statistics_result_path { get; set; } = "Default";
            public int ticks_per_stats_snapshot { get; set; } = 10;
            public bool populate_adjacency_and_comm_groups_every_tick { get; set; } = false;
            public int ticks_before_exploration_heatmap_cold { get; set; } = 2400; // 10*60*4
            public int ticks_before_coverage_heatmap_cold { get; set; } = 2400;    // 10*60*4

            public override string ToString() {
                return $"{nameof(logic_ticks_delta_millis)}: {logic_ticks_delta_millis}, {nameof(physics_ticks_per_logic_update)}: {physics_ticks_per_logic_update}, {nameof(draw_communication)}: {draw_communication}, {nameof(show_environment_tags)}: {show_environment_tags}, {nameof(should_write_csv_results)}: {should_write_csv_results}, {nameof(statistics_result_path)}: {statistics_result_path}, {nameof(ticks_per_stats_snapshot)}: {ticks_per_stats_snapshot}, {nameof(populate_adjacency_and_comm_groups_every_tick)}: {populate_adjacency_and_comm_groups_every_tick}, {nameof(ticks_before_exploration_heatmap_cold)}: {ticks_before_exploration_heatmap_cold}, {nameof(ticks_before_coverage_heatmap_cold)}: {ticks_before_coverage_heatmap_cold}";
            }
        }
            
            
        [SuppressMessage("ReSharper", "MemberHidesStaticFromOuterClass")]
        public class RobotConstraintsType {
            public float broadcast_range { get; set; }
            public bool broadcast_blocked_by_walls { get; set; }
            public float sense_nearby_agents_range { get; set; }
            public bool sense_nearby_agents_blocked_by_walls { get; set; }
            public bool automatically_update_slam { get; set; }
            public int slam_update_interval_in_ticks { get; set; }
            public int slam_sync_interval_in_ticks { get; set; }
            public float slam_position_inaccuracy { get; set; }
            public bool distribute_slam { get; set; }
            public float environment_tag_read_range { get; set; }
            public float slam_raytrace_range { get; set; }
            public float relative_move_speed { get; set; }
            public float agent_relative_size { get; set; }

            public override string ToString() {
                return $"{nameof(broadcast_range)}: {broadcast_range}, {nameof(broadcast_blocked_by_walls)}: {broadcast_blocked_by_walls}, {nameof(sense_nearby_agents_range)}: {sense_nearby_agents_range}, {nameof(sense_nearby_agents_blocked_by_walls)}: {sense_nearby_agents_blocked_by_walls}, {nameof(automatically_update_slam)}: {automatically_update_slam}, {nameof(slam_update_interval_in_ticks)}: {slam_update_interval_in_ticks}, {nameof(slam_sync_interval_in_ticks)}: {slam_sync_interval_in_ticks}, {nameof(slam_position_inaccuracy)}: {slam_position_inaccuracy}, {nameof(distribute_slam)}: {distribute_slam}, {nameof(environment_tag_read_range)}: {environment_tag_read_range}, {nameof(slam_raytrace_range)}: {slam_raytrace_range}, {nameof(relative_move_speed)}: {relative_move_speed}, {nameof(agent_relative_size)}: {agent_relative_size}";
            }
        }

        public class EndCriteriaType {
            public float? coverage_percent { get; set; } = null;
            public float? exploration_percent { get; set; } = null;

            public override string ToString() {
                return $"{nameof(coverage_percent)}: {coverage_percent}, {nameof(exploration_percent)}: {exploration_percent}";
            }
        }

        public class SpawnTogetherType {
            public int? suggested_starting_point_x { get; set; } = null;
            public int? suggested_starting_point_y { get; set; } = null;

            public bool HasSuggestedStartingPoint => this. suggested_starting_point_x != null && this.suggested_starting_point_y != null;

            public override string ToString() {
                return $"{nameof(suggested_starting_point_x)}: {suggested_starting_point_x}, {nameof(suggested_starting_point_y)}: {suggested_starting_point_y}, {nameof(HasSuggestedStartingPoint)}: {HasSuggestedStartingPoint}";
            }
        }

        public class RobotSpawnConfigType {
            public bool? biggest_room { get; set; } = null;
            public SpawnTogetherType spawn_together { get; set; } = null;
            public bool? spawn_at_hallway_ends { get; set; } = null;

            public override string ToString() {
                return $"{nameof(biggest_room)}: {biggest_room}, {nameof(spawn_together)}: {spawn_together}, {nameof(spawn_at_hallway_ends)}: {spawn_at_hallway_ends}";
            }
        }

        public class CaveConfigType {
            public int smoothing_runs { get; set; } = 4;
            public int connection_passage_width { get; set; } = 4;
            public int random_fill_percent { get; set; } = 40;
            public int wall_threshold_size { get; set; } = 10;
            public int room_threshold_size { get; set; } = 10;

            public override string ToString() {
                return $"{nameof(smoothing_runs)}: {smoothing_runs}, {nameof(connection_passage_width)}: {connection_passage_width}, {nameof(random_fill_percent)}: {random_fill_percent}, {nameof(wall_threshold_size)}: {wall_threshold_size}, {nameof(room_threshold_size)}: {room_threshold_size}";
            }
        }

        public class BuildingConfigType {
            public int max_hall_in_percent { get; set; } = 20;
            public int min_room_side_length { get; set; } = 6;
            public int door_width { get; set; } = 2;
            public int door_padding { get; set; } = 2;
            public int room_split_chance { get; set; } = 85;

            public override string ToString() {
                return $"{nameof(max_hall_in_percent)}: {max_hall_in_percent}, {nameof(min_room_side_length)}: {min_room_side_length}, {nameof(door_width)}: {door_width}, {nameof(door_padding)}: {door_padding}, {nameof(room_split_chance)}: {room_split_chance}";
            }
        }

        public class MapType {
            public float wall_height { get; set; }
            public int width_in_tiles { get; set; }
            public int height_in_tiles { get; set; }
            public int border_size { get; set; } = 1;
            public BuildingConfigType building_config { get; set; } = null;
            public CaveConfigType cave_config { get; set; } = null;

            public override string ToString() {
                return $"{nameof(wall_height)}: {wall_height}, {nameof(width_in_tiles)}: {width_in_tiles}, {nameof(height_in_tiles)}: {height_in_tiles}, {nameof(border_size)}: {border_size}, {nameof(building_config)}: {building_config}, {nameof(cave_config)}: {cave_config}";
            }
        }
        
        
    }
}