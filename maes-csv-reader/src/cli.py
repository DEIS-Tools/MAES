# Copyright 2024 MAES
# 
# This file is part of MAES
# 
# MAES is free software: you can redistribute it and/or modify it under
# the terms of the GNU General Public License as published by the
# Free Software Foundation, either version 3 of the License, or (at your option)
# any later version.
# 
# MAES is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
# or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
# Public License for more details.
# 
# You should have received a copy of the GNU General Public License along
# with MAES. If not, see http:#www.gnu.org/licenses/.
# 
# Contributors: Rasmus Borrisholt Schmidt, Andreas Sebastian Sørensen, Thor Beregaard
# 
# Original repository: https:#github.com/Molitany/MAES

import utility
import os

def cli(args, options):
    """
    View component for Command-Line Interface
    """
    current_arg = ""
    help_requested = False
    for arg in args:
        match arg:
            case "-x" | "--x-column-name":
                current_arg = "x"
            case "-y" | "--y-column-name":
                current_arg = "y"
            case "-t" | "--plot-type":
                current_arg = "plottype"
            case "-a" | "--absolute-path":
                current_arg = "abspath"
            case "-f" | "--filenames":
                current_arg = "filename"
            case "-i" | "--file-identifiers":
                current_arg = "fileid"
            case "-d" | "--directories":
                current_arg = "dir"
            case "-e" | "--export-file-name":
                current_arg = "exportname"
            case "-p" | "--show-plot":
                current_arg = "showplot"
            case "-h" | "--help":
                print_help(options.errors)
                help_requested = True
                break
            case _:
                if arg[0] == "-":
                    options.append_error(f"unknown argument {arg}")
                    continue

                add_args_to_options(current_arg, arg, options)

    if help_requested == False:
        check_required_input(options)

    if len(options.errors) != 0:
        print_help(options.errors)

def add_args_to_options(current_arg, arg, options):
    """
    Function to set the options according to CLI input
    """
    match current_arg:
        case "x":
            if options.x_column_name != "":
                options.append_error("x was given multiple names")
            else:
                options.set_x(arg)

        case "y":
            if options.y_column_name != "":
                options.append_error("y was given multiple names")
            else:
                options.set_y(arg)

        case "plottype":
            options.set_plot_type(arg)

        case "abspath":
            if arg == "false" or arg == "False" or arg == "0":
                options.set_abs_path(False)
            elif arg == "true" or arg == "True" or arg == "1":
                options.set_abs_path(True)
            else:
                options.append_error(f"--absolute-path (or -a) was given argument {arg}, this is invalid!")

        case "filename":
            options.append_filename(arg)

        case "fileid":
            options.append_fileid(arg)

        case "dir":
            options.append_directory(arg)
            for f in sorted(utility.get_dir_contents(arg)):
                options.append_filename(os.path.join(arg, f))

        case "exportname":
            if options.export_file_name != "":
                options.append_error("export file was given multiple names")
            else:
                options.set_export_name(arg)

        case "showplot":
            if arg == "false" or arg == "False" or arg == "0":
                options.set_show_plot(False)
            elif arg == "true" or arg == "True" or arg == "1":
                options.set_show_plot(True)
            else:
                options.append_error(f"--show-plot (or -p) was given argument {arg}, this is invalid!")

def check_required_input(options):
    """
    Checks if the required input has been provided to the options class from the CLI.
    """
    missing_options = []
    if options.x_column_name == "":
        missing_options.append("-x")
    if options.y_column_name == "":
        missing_options.append("-y")
    if len(options.file_names) == 0 and len(options.directories) == 0:
        missing_options.append("-f or -d")
    if len(options.file_identifiers) != 0:
        if len(options.file_identifiers) != len(options.file_names):
            options.file_identifiers = options.file_names
            #options.append_error(f"File identifiers were provided, but does not have the same amount as file names.")
    else:
        options.file_identifiers = options.file_names

    if len(missing_options) != 0:
        options.append_error(f"Missing options: {missing_options}")


def print_help(errors: [str]):
    """
    Prints the help text in the CLI.
    """
    print("You can write -h or --help for help.")
    print("This program expects you to set various variables through the command line.")
    print("-x <x label> or --x-column-name <x label> for the name of the column of the 1st axis. (Required)")
    print("-y <y label> or --y-column-name <y label> for the name of the column of the 2nd axis. (Required)")
    print("-t <base/cactus> or --plot-type <base/cactus> to define which type of plot you wish to output (Optional, default is base)")
    print("-a <true/false> or --absolute path <true/false> to define whether or not to use the file and directory paths as absolute. (Optional, false is default)")
    print("-f <file 1> ... <file n> or --filenames <file 1> ... <file n> to set which files to add to the plot. (Either -f or -d is required)")
    print("-d <dir 1> ... <dir n> or --directories <dir 1> ... dir n> to set which directories to add to the plot (Either -f or -d is required)")
    print("-i <file 1> ... <file n> or --file-identifier <file 1> ... <file n> to set the name of the plots as seen in final product. (Optional, filenames will be used if omitted)")
    print("-e <name> or --export-file-name <name> to set the name of the export file. (Optional, but required if you want to export)")
    print("-p <true/false> or --show-plot <true/false> to show the plot or not. (Optional, false as default)")
    print("Example: python main.py -x Tick -y Tick -d testdir1 testdir2 -t cactus -e experiment1 -p false")
    if len(errors) != 0:
        print(f"Program failed to execute, errors encountered: ")
        for error in errors:
            print(error)
