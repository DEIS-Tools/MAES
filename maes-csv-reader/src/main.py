import sys
import os

import csvreader
import plotter
import plot
import table as t
import exporter
import options as ops
import cli
import utility

def main(args):
    options = ops.Options()

    cli.cli(args, options)

    csv_files = []
    for i, f in enumerate(options.file_names):
        csv_file = csvreader.CsvReader({options.file_identifiers[i]}, f, options.absolute_path)
        csv_files.append(csv_file)

    if options.plot_type == "base":
        plots = make_base_plots(csv_files, options)
    elif options.plot_type == "cactus":
        plots = make_cactus_plots(csv_files, options)
        table = make_table(csv_files, options)
    elif options.plot_type == "ratio":
        plots = make_ratio_plots(csv_files, options)

    if options.show_plot == True:
        show_plot(plots, options)
    if options.export_file_name != "":
        export_to_tikz(plots, options)
        if options.plot_type == "cactus":
            export_to_latex_table(table, options)

    return 0

def make_table(csv_files, options):
    table = t.Table()

    separated_csvs = separate_plots_by_directory(csv_files, options)
    fastest_success_name = ''

    for directory in separated_csvs:
        name = ''
        average_sum = 0.0
        successes = 0.0
        timeouts = 0.0
        success_rate = 0.0
        fastest_success = 36000
        slowest_success = 0
        data_point_amount = 0
        for csv in directory:
            last_tick = int(csv.get_last_column_element('Tick'))
            average_sum += last_tick
            data_point_amount += 1
            name = csv.object_name

            if last_tick > slowest_success and last_tick != 36000:
                slowest_success = last_tick
                slowest_success_name = csv.file_name
            if last_tick < fastest_success:
                fastest_success = last_tick
                fastest_success_name = csv.file_name
            if last_tick == 36000:
                timeouts += 1
            else:
                successes += 1
        print(f"fastest map: {fastest_success_name}")
        print(f"slowest map: {slowest_success_name}")

        if (successes+timeouts) > 0:
            success_rate = successes/(successes+timeouts)
            average = average_sum/data_point_amount
            name = ', '.join(name).split(os.sep)[0]
            table.add_row(t.TableRow(name, average, successes, timeouts, success_rate, fastest_success, slowest_success))

    return table


def make_base_plots(csv_files, options):
    """
    Function to create the base x,y plots.
    """
    x_name = ''
    y_name = ''
    plots = []
    for csv in csv_files:
        x_name, temp_x = csv.get_column_by_name(options.x_column_name)
        y_name, temp_y = csv.get_column_by_name(options.y_column_name)
        plots.append(plot.Plot(csv.object_name, temp_x, temp_y, x_name, y_name))

    return plots

def make_cactus_plots(csv_files, options):
    """
    Function to create cactus plots from the csv files.
    """
    x_name = 'instance'
    y_name = options.y_column_name
    plots = []
    separated_csvs = separate_plots_by_directory(csv_files, options)
    for directory in separated_csvs:
        temp_x = []
        temp_y = []
        name = ''
        for csv in directory:
            temp_y.append(csv.get_last_column_element(options.y_column_name))
            name = csv.object_name

        temp_x = range(1, len(temp_y)+1)
        temp_y.sort()
        name = ', '.join(name).split(os.sep)[0]
        plots.append(plot.Plot(name, temp_x, temp_y, x_name, y_name))

    return plots

def make_ratio_plots(csv_files, options):
    """
    Function to create ratio plots from the csv files.
    """
    x_name = 'instance'
    y_name = options.y_column_name
    temp_plots = []
    plots = []
    separated_csvs = separate_plots_by_directory(csv_files, options)
    if len(separated_csvs) != 2:
        raise Exception("Ratio plots only works with two folders! (at the moment)")

    for directory in separated_csvs:
        temp_y = []
        for csv in directory:
            temp_y.append(csv.get_last_column_element(options.y_column_name))
        temp_plots.append(temp_y)

    if len(separated_csvs) != 2:
        raise Exception("Ratio plots only works with two folders! (at the moment)")

    if len(temp_plots[0]) != len(temp_plots[1]):
        raise Exception("Different amount of csv files in the directories!")

    temp_y = []
    largest_diff = 0
    largest_diff_seed = ""
    for i in range(0, len(temp_plots[0])):
        temp_y.append(temp_plots[0][i] - temp_plots[1][i])
        if temp_plots[0][i] < temp_plots[1][i]:
            pass
            #print(f"greed: {separated_csvs[0][i].file_name} is faster")
        elif temp_plots[0][i] > temp_plots[1][i]:
            print(f"minotaur: {separated_csvs[1][i].file_name} is faster")
            print(f"first: {temp_plots[0][i]}, second: {temp_plots[1][i]}")
            if (temp_plots[0][i] - temp_plots[1][i]) > largest_diff:
                largest_diff = temp_plots[0][i] - temp_plots[1][i]
                largest_diff_seed = separated_csvs[1][i].file_name
        else:
            print("equal speed")
        #print(f"result: {temp_y[i]}")
    print(f"largest difference is {largest_diff} on map {largest_diff_seed}")

    temp_y.sort()
    temp_x = range(0, len(temp_y))
    plots.append(plot.Plot('versus', temp_x, temp_y, x_name, y_name))

    return plots

def separate_plots_by_directory(csv_files, options):
    """
    Function to separate the different csv files according to which directory they're in.
    Returns a list of lists of csv files, sorted according to their directory.
    """
    separated_plots = [[]]
    i = 0

    for directory in options.directory_names:
        #print(f"in directory: {directory}")
        for csv in csv_files:
            #print(csv.file_name)
            if directory in csv.file_name:
                if csv.file_name[len(directory)] == f"{os.path.sep}":
                    separated_plots[i].append(csv)
        separated_plots.append([])
        i += 1
    separated_plots.pop()
    return separated_plots

def export_to_tikz(plots, options):
    """
    Exports a tikz plot for use in LaTeX
    """
    export = exporter.Exporter(plots, None)
    export.export_to_tikz(options)
    export.save_to_file(options.export_file_name)

def export_to_latex_table(table, options):
    export = exporter.Exporter(None, table)
    export.export_latex_table(options)
    export.save_to_file(options.export_file_name)


def show_plot(plots, options):
    """
    Shows the plot after launching the program, for testing purposes mostly.
    """
    x_name = plots[0].x_name
    y_name = plots[0].y_name
    x = []
    y = []
    for p in plots:
        temp_x = p.xcords
        temp_y = p.ycords
        x.append(temp_x)
        y.append(temp_y)

    plot = plotter.Plotter(plots)
    plot.show()

if __name__ == '__main__':
    """
    Code to initalise main and throw an exception if 0 is not returned.
    """
    return_code = main(sys.argv)
    if return_code != 0:
        raise Exception(f'main returned {return_code}')
