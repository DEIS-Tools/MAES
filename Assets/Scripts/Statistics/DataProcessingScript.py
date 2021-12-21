import numpy
import matplotlib.pyplot as plt
import numpy as np
from numpy import genfromtxt, ndarray
import os

explored = 'explored'
coverage = 'covered'

ssb = 'SSB'
lvd = 'LVD'
rbw = 'RBW'
tnf = 'TNF'
lvd_long_range = 'LVD-long-range'
tnf_global = 'TNF-Global'

# Map configurations
building50 = '-building-50x50-hallway'
building100 = '-building-100x100-hallway'
building200 = '-building-200x200-hallway'
cave50 = '-cave-50x50-spawnTogether'
cave100 = '-cave-100x100-spawnTogether'
cave200 = '-cave-200x200-spawnTogether'

performance_algorithms = [ssb, lvd, rbw, tnf]
performance_map_configurations = [building50, building100, building200, cave50, cave100, cave200]

PERFORMANCE_MAX_DPS = 3596
ticks = [i * 10 for i in range(PERFORMANCE_MAX_DPS)]

# Configurations for robot count
cave75 = '-cave-75x75'
building75 = '-building-75x75'
all_robot_counts = [1, 5, 10, 15, 20, 25, 30, 35, 40]

robot_count_map_configurations = [cave75, building75]
ROBOT_COUNT_MAX_DPS = 1190
robot_count_ticks = [i * 10 for i in range(ROBOT_COUNT_MAX_DPS)]


class PerformanceSummary:
    def __init__(self, data: ndarray) -> None:
        self.avg = data[0]
        self.max = data[1]
        self.min = data[2]
        self.fastest_run = data[3]
        self.slowest_run = data[4]
        self.fastest_five_avg = data[5]
        self.slowest_five_avg = data[6]


performance_summary_dict = dict[str, dict[(str, str), PerformanceSummary]]


def generate_performance_summaries(save_files=False, algorithms=performance_algorithms, map_configs = performance_map_configurations) -> performance_summary_dict:
    column_names = ['avg', 'max', 'min', 'fastest', 'slowest', 'five_fastest_avg', 'five_slowest_avg']
    all_configurations = dict()

    ticks = [int(i) for i in range(3596)]
    for map_config in map_configs:
        all_configurations[map_config] = dict()
        this_dict = all_configurations[map_config]

        exploration_summaries = [ticks]
        coverage_summaries = [ticks]
        titles = 'ticks,'
        format_string = '%i,'
        for algorithm in algorithms:
            name = algorithm + map_config
            coverage_summary, exploration_summary = calculate_summary([name])
            exploration_summaries.extend(exploration_summary)
            coverage_summaries.extend(coverage_summary)

            # Add titles and format options
            for col_name in column_names:
                titles += algorithm + '_' + col_name + ','
                format_string += '1.3f,'

            # Save to dict
            this_dict[(algorithm, explored)] = PerformanceSummary(exploration_summary)
            this_dict[(algorithm, coverage)] = PerformanceSummary(coverage_summary)

        if save_files:
            exploration_summaries = np.swapaxes(exploration_summaries, 0, 1)
            coverage_summaries = np.swapaxes(coverage_summaries, 0, 1)
            np.savetxt('Summary/' + map_config[1:] + '-exploration-summary.csv', exploration_summaries,
                       delimiter=',', comments='', fmt=format_string[:-1], header=titles[:-1])

            np.savetxt('Summary/' + map_config[1:] + '-coverage-summary.csv', coverage_summaries, delimiter=',',
                       fmt=format_string[:-1],
                       header=titles, comments='')

    return all_configurations


def calculate_summary(file_names):
    seeds = 20
    for file_prefix in file_names:
        coverage, exploration = fetch_data(file_prefix, seeds)

        # Overall:
        mean_coverage = np.mean(coverage, 0)
        mean_exploration = np.mean(exploration, 0)

        max_coverage = np.amax(coverage, 0)
        min_coverage = np.amin(coverage, 0)
        max_explored = np.amax(exploration, 0)
        min_explored = np.amin(exploration, 0)

        # Best and worst:
        ordered_by_coverage_descending = order_runs(coverage, 99.5)
        ordered_by_exploration_descending = order_runs(exploration, 100)

        best_coverage_run = coverage[ordered_by_coverage_descending[0][0]]
        worst_coverage_run = coverage[ordered_by_coverage_descending[seeds-1][0]]
        best_explored_run = exploration[ordered_by_exploration_descending[0][0]]
        worst_explored_run = exploration[ordered_by_exploration_descending[seeds-1][0]]

        five_best_coverage_run = np.mean([coverage[i] for i, _, _ in ordered_by_coverage_descending[:5]], 0)
        five_worst_coverage_run = np.mean([coverage[i] for i, _, _ in ordered_by_coverage_descending[-5:]], 0)
        five_best_explored_run = np.mean([exploration[i] for i, _, _ in ordered_by_exploration_descending[:5]], 0)
        five_worst_explored_run = np.mean([exploration[i] for i, _, _ in ordered_by_exploration_descending[-5:]], 0)

        #  avg, max, min, fastest, slowest, avg five fastest, avg five slowest
        return ([mean_coverage, max_coverage, min_coverage, best_coverage_run, worst_coverage_run, five_best_coverage_run, five_worst_coverage_run],
                [mean_exploration, max_explored, min_explored, best_explored_run, worst_explored_run, five_best_explored_run, five_worst_explored_run])

    # plt.title('Exploration over time')
    # plt.ylabel('Explored %')
    # plt.xlabel('Ticks')
    # plt.legend()
    # plt.show()

    #np.savetxt(scenario_name + '-coverage.csv', coverage, delimiter=',', fmt='%f')
    #np.savetxt(scenario_name + '-exploration.csv', exploration, delimiter=',', fmt='%f')





def order_runs(data: ndarray, max_value):
    # Scores a single run based on finish time and
    def finish_time_tuple(index):
        col = data[index]
        highest_value = col.max()
        if highest_value == max_value:
            # finished before sim end
            finish_time = np.where(col == highest_value)[0][0]
        else:
            # Never finished
            finish_time = data.shape[1]
        return index, finish_time, highest_value

    entries = [finish_time_tuple(i) for i in range(data.shape[0])]
    entries.sort(key=lambda e: (e[1], -e[2]))
    return entries


def fetch_data(file_prefix, seeds):
    exploration_data = np.full((seeds, 3596), 100.0)
    coverage_data = np.full((seeds, 3596), 99.5)
    counter = 0
    for file in os.listdir("Data"):
        if file.startswith(file_prefix):
            file_data = genfromtxt('Data/' + file, delimiter=',', skip_header=1).swapaxes(0, 1)
            numpy.insert(coverage_data, counter, file_data[1])

            data_point_count = file_data.shape[1]
            coverage_data[counter][0:data_point_count] = file_data[1]
            # Special case for tnf algorithm
            if file.startswith(tnf):
                max_covered = file_data[1][data_point_count-1]
                coverage_data[counter][data_point_count:] = max_covered

            exploration_data[counter][0:file_data.shape[1]] = file_data[2]
            counter += 1
            if counter == seeds:
                break

    return coverage_data, exploration_data


# -------------------------------- Robot Count Scenario ----------------------------------
class RobotCountSummary:
    def __init__(self, exploration_rate, coverage_rate) -> None:
        self.exploration_per_minute = exploration_rate
        self.coverage_per_minute = coverage_rate


def fetch_robot_count_data(file_prefix) -> [RobotCountSummary]:
    summaries = []
    for file in os.listdir("DataRobotCount"):
        if file.startswith(file_prefix):
            file_data = genfromtxt('DataRobotCount/' + file, delimiter=',', skip_header=1).swapaxes(0, 1)

            max_coverage_index = np.argmax(file_data[1])
            max_coverage_tick = 10 * max_coverage_index
            max_coverage = file_data[1][max_coverage_index]
            coverage_rate = (max_coverage - file_data[1][0]) / (max_coverage_tick / (10 * 60))

            max_exploration_index = np.argmax(file_data[2])
            max_exploration_tick = 10 * max_exploration_index
            max_exploration = file_data[2][max_exploration_index]
            exploration_rate = (max_exploration - file_data[2][0]) / (max_exploration_tick / (10 * 60))

            summaries.append(RobotCountSummary(exploration_rate, coverage_rate))

    return summaries


RobotCountDict = dict[(str, str), RobotCountSummary]


def generate_robot_count_summaries(algorithms: [str], robot_counts: [int], scenario) -> RobotCountDict:
    robot_count_dict = dict()
    for alg in algorithms:
        for count in robot_counts:
            file_prefix = f'{alg}{scenario}-robots{count}-'
            summaries = fetch_robot_count_data(file_prefix)
            avg_exploration_rate = 0
            avg_coverage_rate = 0
            for summary in summaries:
                avg_exploration_rate += summary.exploration_per_minute
                avg_coverage_rate += summary.coverage_per_minute
            avg_exploration_rate /= len(summaries)
            avg_coverage_rate /= len(summaries)
            robot_count_dict[(alg, count)] = RobotCountSummary(avg_exploration_rate, avg_coverage_rate)

    return robot_count_dict


# -------------- Plotting ------------------
chart_colors = {lvd: 'red', rbw: 'blue', ssb: 'green', tnf: 'pink', lvd_long_range: 'purple', tnf_global: 'purple'}
line_styles = {coverage: 'solid', explored: 'dashed'}


def plot_summary(data: performance_summary_dict, map_config: str, title: str, data_chooser, max_x=36000):
    for (alg, measure) in data[map_config].keys():
        summary = data[map_config][(alg, measure)]
        plt.plot(ticks, data_chooser(summary), color=chart_colors[alg],
                 label=f'{alg} {measure}', linestyle=line_styles[measure])

    plt.title(title, fontsize=15, pad=20)
    plt.xlabel('Minutes', fontsize=15, labelpad=5)
    plt.ylabel('Area in %', fontsize=15)
    plt.legend()
    plt.xticks(np.arange(13) * 3000, [0, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60], fontsize=14)
    #plt.xticks(np.arange(10) * 10 * 60, [1, 2, 3, 4, 5, 6, 7, 8, 9, 10], fontsize=14)
    plt.yticks(fontsize=14)
    plt.xlim(0, max_x)
    plt.ylim(0, 101)

    plt.show()


def plot_worst_case_explored(data: performance_summary_dict, map_config: str, algorithms: [str], title: str, max_x=36000):
    for alg in algorithms:
        summary = data[map_config][(alg, explored)]
        plt.plot(ticks, summary.slowest_run, color=chart_colors[alg], label=f'Slowest {alg} run', linestyle='dotted')
        plt.plot(ticks, summary.avg, color=chart_colors[alg], label=f'Averaged {alg}', linestyle='dashed')

    plt.title(title, fontsize=15, pad=20)
    plt.xlabel('Minutes', fontsize=15, labelpad=5)
    plt.ylabel('Explored Area in %', fontsize=15)
    plt.legend()
    plt.xticks(np.arange(13) * 3000, [0, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60], fontsize=14)
    plt.yticks(fontsize=14)
    plt.xlim(0, max_x)
    plt.ylim(0, 101)

    plt.show()


def plot_worst_case_covered(data: performance_summary_dict, map_config: str, algorithms: [str], title: str, max_x=36000):
    for alg in algorithms:
        summary = data[map_config][(alg, coverage)]
        plt.plot(ticks, summary.slowest_run, color=chart_colors[alg], label=f'Slowest {alg} run', linestyle='dotted')
        plt.plot(ticks, summary.avg, color=chart_colors[alg], label=f'Averaged {alg}', linestyle='solid')

    plt.title(title, fontsize=15, pad=20)
    plt.xlabel('Minutes', fontsize=15, labelpad=5)
    plt.ylabel('Covered Area in %', fontsize=15)
    plt.legend()
    plt.xticks(np.arange(13) * 3000, [0, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60], fontsize=14)
    plt.yticks(fontsize=14)
    plt.xlim(0, max_x)
    plt.ylim(0, 101)

    plt.show()


def plot_robot_count(robot_count_dict: RobotCountDict, plotted_counts: [int], title: str, algorithms=performance_algorithms):
    bar_width = 0.15
    x_ticks = np.arange(len(plotted_counts))
    for index, alg in enumerate(algorithms):
        plt.bar([x + index * bar_width for x in x_ticks],
                [robot_count_dict[(alg, count)].exploration_per_minute for count in plotted_counts],
                color=chart_colors[alg], edgecolor='black', label=alg,
                width=bar_width)

    plt.xticks([r + bar_width for r in range(len(plotted_counts))],
               plotted_counts)

    plt.title(title, fontsize=15, pad=20)
    plt.ylabel('Area % Explored per Minute', fontsize=15, labelpad=5)
    plt.xlabel('Number of Agents', fontsize=15)
    plt.legend()
    plt.show()


def plot_robot_count_rate_per_robot(robot_count_dict: RobotCountDict, plotted_counts: [int], title: str, algorithms=performance_algorithms):
    data = []
    bar_width = 0.15
    x_ticks = np.arange(len(plotted_counts))
    for index, alg in enumerate(algorithms):
        plt.bar([x + index * bar_width for x in x_ticks],
                [robot_count_dict[(alg, count)].exploration_per_minute / count for count in plotted_counts],
                color=chart_colors[alg], edgecolor='black', label=alg,
                width=bar_width)

    plt.xticks([r + bar_width for r in range(len(plotted_counts))],
               plotted_counts)

    plt.title(title, fontsize=15, pad=20)
    plt.ylabel('Area % Explored per Minute per Robot', fontsize=15, labelpad=5)
    plt.xlabel('Number of Agents', fontsize=15)
    plt.legend()
    plt.show()

def plot_robot_count_coverage(robot_count_dict: RobotCountDict, plotted_counts: [int], title: str, algorithms=performance_algorithms):
    bar_width = 0.15
    x_ticks = np.arange(len(plotted_counts))
    for index, alg in enumerate(algorithms):
        plt.bar([x + index * bar_width for x in x_ticks],
                [robot_count_dict[(alg, count)].coverage_per_minute for count in plotted_counts],
                color=chart_colors[alg], edgecolor='black', label=alg,
                width=bar_width)

    plt.xticks([r + bar_width for r in range(len(plotted_counts))],
               plotted_counts)

    plt.title(title, fontsize=15, pad=20)
    plt.ylabel('Area % Covered per Minute', fontsize=15, labelpad=5)
    plt.xlabel('Number of Agents', fontsize=15)
    plt.legend()
    plt.show()


def plot_robot_count_coverage_rate_per_robot(robot_count_dict: RobotCountDict, plotted_counts: [int], title: str, algorithms=performance_algorithms):
    data = []
    bar_width = 0.15
    x_ticks = np.arange(len(plotted_counts))
    for index, alg in enumerate(algorithms):
        plt.bar([x + index * bar_width for x in x_ticks],
                [robot_count_dict[(alg, count)].coverage_per_minute / count for count in plotted_counts],
                color=chart_colors[alg], edgecolor='black', label=alg,
                width=bar_width)

    plt.xticks([r + bar_width for r in range(len(plotted_counts))],
               plotted_counts)

    plt.title(title, fontsize=15, pad=20)
    plt.ylabel('Area % Covered per Minute per Robot', fontsize=15, labelpad=5)
    plt.xlabel('Number of Agents', fontsize=15)
    plt.legend()
    plt.show()



if __name__ == '__main__':
     collected_data = generate_performance_summaries(save_files=False)
    # -------------- Performance Summary -------------
    plot_summary(collected_data,
                 map_config=building100,
                 title='Average coverage/exploration for 200x200 building maps',
                 data_chooser=lambda summary: summary.avg)

    # ------ LVD Long Range -------
    lvd_range_algs = [lvd, lvd_long_range]
    collected_data = generate_performance_summaries(save_files=False, algorithms=lvd_range_algs, map_configs=[building200, cave200])
    plot_summary(collected_data, map_config=building200,
                    title='LVD Visibility Range Comparison for 200x200 Building',
                    data_chooser=lambda summary: summary.avg)
    
    plot_summary(collected_data, map_config=cave200,
                  title='LVD Visibility Range Comparison for 200x200 Cave',
                  data_chooser=lambda summary: summary.avg)


    # ------ WORST CASE -----
    plot_worst_case_explored(collected_data, map_config=building200, algorithms=performance_algorithms,
                     title='Slowest Run and Averaged Explored for 200x200 building')
    
    plot_worst_case_explored(collected_data, map_config=cave200, algorithms=performance_algorithms,
                     title='Slowest run and Averaged run for 200x200 cave')
    
    plot_worst_case_covered(collected_data, map_config=building200, algorithms=performance_algorithms,
                     title='Slowest run and Averaged run for 200x200 building')
    
    plot_worst_case_covered(collected_data, map_config=cave200, algorithms=performance_algorithms,
                    title='Slowest run and Averaged run for 200x200 cave')

    # ------- TNF Global Communication -----
    tnf_global_data = generate_performance_summaries(save_files=False, algorithms=[tnf, tnf_global], map_configs=[cave200, building200])
    plot_summary(tnf_global_data,
                 map_config=building200,
                 title='Global Communication TNF in 200x200 building',
                 data_chooser=lambda summary: summary.fastest_run)
    
    plot_summary(tnf_global_data,
                 map_config=cave200,
                 title='Global Communication TNF in 200x200 cave',
                 data_chooser=lambda summary: summary.fastest_run)

    # -------------- Robot count --------------
    robot_count_dict = generate_robot_count_summaries(algorithms=performance_algorithms, scenario=cave75, robot_counts=all_robot_counts)
    plot_robot_count(robot_count_dict, plotted_counts=[1, 10, 20, 30, 40], algorithms=performance_algorithms,
                     title='Exploration Rate by Robot Count in 75x75 Building')
    plot_robot_count_rate_per_robot(robot_count_dict, plotted_counts=[1, 10, 20, 30, 40], algorithms=performance_algorithms,
                                    title='Exploration Rate Per Robot in 75x75 Building')
    plot_robot_count_coverage(robot_count_dict, plotted_counts=[1, 10, 20, 30, 40], algorithms=performance_algorithms,
                              title='Coverage Rate by Robot Count in 75x75 Building')
    plot_robot_count_coverage_rate_per_robot(robot_count_dict, plotted_counts=[1, 10, 20, 30, 40],
                                             algorithms=performance_algorithms,
                                              title='Coverage Rate Per Robot in 75x75 Building')

    robot_count_dict = generate_robot_count_summaries(algorithms=performance_algorithms, scenario=cave75,
                                                      robot_counts=all_robot_counts)
    plot_robot_count(robot_count_dict, plotted_counts=[1, 10, 20, 30, 40], algorithms=performance_algorithms,
                     title='Exploration Rate by Robot Count in 75x75 Cave')
    plot_robot_count_rate_per_robot(robot_count_dict, plotted_counts=[1, 10, 20, 30, 40], algorithms=performance_algorithms,
                                     title='Exploration Rate Per Robot in 75x75 Cave')
    plot_robot_count_coverage(robot_count_dict, plotted_counts=[1, 10, 20, 30, 40], algorithms=performance_algorithms,
                      title='Coverage Rate by Robot Count in 75x75 Cave')
    plot_robot_count_coverage_rate_per_robot(robot_count_dict, plotted_counts=[1, 10, 20, 30, 40],
                                    algorithms=performance_algorithms,
                                    title='Coverage Rate Per Robot in 75x75 Cave')


